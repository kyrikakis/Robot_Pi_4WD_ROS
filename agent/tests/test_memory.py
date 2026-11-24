"""Memory tests with a fake model and a fake embedder (bag of words)."""

import asyncio
import json
from types import SimpleNamespace as NS

import numpy as np
from rainbow_agent.memory import Memory, parse_summary, transcript

VOCAB = ['mia', 'dinosaurs', 'likes', 'kitchen', 'blue', 'colour', 'favourite', 'name',
         'cat', 'luna', 'birthday', 'robot', 'played']


async def embed(texts):
    """Bag-of-words vectors: similar wording, similar vectors."""
    out = []
    for t in texts:
        words = t.lower().replace('?', ' ').replace('.', ' ').replace("'", ' ').split()
        v = np.array([words.count(w) for w in VOCAB], dtype=np.float32) + 1e-3
        out.append(v)
    return out


class FakeLLM:

    def __init__(self, *replies):
        self.replies = list(replies)
        self.prompts = []
        self.chat = NS(completions=NS(create=self.create))

    async def create(self, **kwargs):
        self.prompts.append(kwargs['messages'][-1]['content'])
        return NS(choices=[NS(message=NS(content=self.replies.pop(0)))])


SESSION = [
    {'role': 'user', 'content': "Hi Rainbow, I'm Mia!"},
    {'role': 'assistant', 'content': 'Hi Mia!'},
    {'role': 'assistant', 'content': '', 'tool_calls': []},
    {'role': 'tool', 'content': '{"ok": true}'},
    {'role': 'user', 'content': '[Robot event] arrived at the kitchen'},
    {'role': 'user', 'content': 'I love dinosaurs. My cat is called Luna. '
                                "Mia's favourite colour is blue."},
]


def memory(tmp_path, *replies):
    return Memory(tmp_path / 'memory.db', FakeLLM(*replies), 'm', embed)


def run(coro):
    return asyncio.run(coro)


def test_transcript_and_parsing():
    text = transcript(SESSION)
    assert text.splitlines() == ["Person: Hi Rainbow, I'm Mia!", 'Rainbow: Hi Mia!',
                                 '[Robot event] arrived at the kitchen',
                                 'Person: I love dinosaurs. My cat is called Luna. '
                                 "Mia's favourite colour is blue."]
    assert parse_summary('Sure! {"summary": "Mia visited.", "facts": ["Mia likes dinosaurs"]}') \
        == ('Mia visited.', ['Mia likes dinosaurs'])
    assert parse_summary('no json here') == ('', [])


def test_store_and_recall(tmp_path):
    m = memory(tmp_path, json.dumps({
        'summary': 'Mia said hi and talked about dinosaurs.',
        'facts': ['Mia likes dinosaurs.', 'Mia has a cat called Luna.']}))
    summary, facts = run(m.store_session(SESSION, started=1.0))
    assert m.counts() == {'episodes': 1, 'facts': 2}
    lines = run(m.recall('Do you remember what dinosaurs Mia likes?'))
    assert 'Fact: Mia likes dinosaurs.' in lines
    assert any(line.startswith(('Earlier:', 'Last time:')) for line in lines)
    # Survives reopening
    m.close()
    again = Memory(tmp_path / 'memory.db', None, 'm', embed)
    assert again.counts() == {'episodes': 1, 'facts': 2}


def test_same_fact_is_updated_not_duplicated(tmp_path):
    m = memory(tmp_path,
               json.dumps({'summary': 'Talked colours.',
                           'facts': ["Mia's favourite colour blue."]}),
               json.dumps({'summary': 'Talked colours again.',
                           'facts': ["Mia's favourite colour is blue."]}))
    run(m.store_session(SESSION, started=1.0))
    run(m.store_session(SESSION, started=2.0))
    assert m.counts() == {'episodes': 2, 'facts': 1}
    assert m.db.execute('SELECT text FROM facts').fetchone()[0] == "Mia's favourite colour is blue."


def test_sessions_without_people_are_not_stored(tmp_path):
    m = memory(tmp_path)
    assert run(m.store_session([{'role': 'user', 'content': '[Robot event] arrived'}], 1.0)) \
        is None
    assert m.counts() == {'episodes': 0, 'facts': 0}
    assert run(m.recall('')) == []


def test_only_facts_people_said_are_kept(tmp_path):
    from rainbow_agent.memory import grounded_in
    said = 'Hi Rainbow! My name is Mia. My cat is called Luna and she is black.'
    assert grounded_in('Mia has a cat named Luna', said)
    assert grounded_in('Luna is black', said)
    assert not grounded_in('Luna likes napping in sunny spots', said)
    assert not grounded_in('Luna', said)                  # too short to be a fact
    m = memory(tmp_path, json.dumps({'summary': 'Mia talked about Luna.',
                                     'facts': ['Mia has a cat named Luna',
                                               'Luna likes chasing balls of light']}))
    session = [{'role': 'user', 'content': said},
               {'role': 'assistant', 'content': 'Luna likes chasing balls of light!'}]
    run(m.store_session(session, started=1.0))
    assert [r[0] for r in m.db.execute('SELECT text FROM facts')] == ['Mia has a cat named Luna']
