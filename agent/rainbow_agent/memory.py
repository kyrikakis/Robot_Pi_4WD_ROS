"""Rainbow's long-term memory: SQLite + embeddings, on the Mac only (docs/brain.md).

  episodes  one per finished session: a short summary, with its embedding
  facts     lasting things worth remembering (names, preferences, things the
            owner asked to remember), with their embeddings

When a session ends, the model summarises it and lists new facts (one call,
JSON). The prompt alone does not stop the model from turning its own guesses
into facts ("Luna likes napping in sunny spots", which nobody said), so a fact
is only kept if most of its words were said by a person (grounded_in) and it
has at least 3 words. A fact nearly the same as a known one replaces it.
When a session starts, its first words are embedded and the closest facts and
episodes, plus the last episode, go into the system prompt.

Embeddings come from Qwen3-Embedding on the second llama-server (port 8181).
Queries get Qwen3-Embedding's instruction prefix; stored texts do not.
"""

import json
import re
import sqlite3
import time

import numpy as np

QUERY_PREFIX = 'Instruct: Given something said to a home robot, retrieve its relevant ' \
               'memories\nQuery: '
SUMMARY_PROMPT = """\
You are Rainbow, a home robot, writing your own private memory of the \
conversation below. Reply with JSON only, no other text:
{"summary": "<1 to 3 short sentences: who talked to you and what happened>",
 "facts": ["<a lasting fact worth remembering>", ...]}
Facts are things still true later: people's names, what they like, places, \
things someone asked you to remember. Only what a Person line says: never take a \
fact from Rainbow's own lines (Rainbow sometimes guesses). Not passing events, \
not the robot's battery or position. Use [] when there are none."""

SCHEMA = """
CREATE TABLE IF NOT EXISTS episodes (
    id INTEGER PRIMARY KEY, started REAL, ended REAL, summary TEXT, embedding BLOB);
CREATE TABLE IF NOT EXISTS facts (
    id INTEGER PRIMARY KEY, created REAL, updated REAL, text TEXT, embedding BLOB);
"""


_FILLER = {'the', 'a', 'an', 'and', 'is', 'are', 'was', 'has', 'have', 'had', 'of', 'to',
           'in', 'on', 'with', 'her', 'his', 'their', 'she', 'he', 'they', 'it', 'its',
           'named', 'called', 'person', "person's"}


def _words(text):
    return re.findall(r"[a-z0-9']+", text.lower())


def grounded_in(fact, said, share=0.6):
    """Return True if at least share of the fact's meaningful words were said."""
    words = [w for w in _words(fact) if w not in _FILLER]
    heard = set(_words(said))
    if len(_words(fact)) < 3 or not words:
        return False
    hits = sum(w in heard or w.rstrip('s') in heard or w + 's' in heard
               or w.rstrip("'s") in heard for w in words)
    return hits / len(words) >= share


def _vector(blob):
    return np.frombuffer(blob, dtype=np.float32)


def _blob(vector):
    v = np.asarray(vector, dtype=np.float32)
    return (v / (np.linalg.norm(v) or 1.0)).tobytes()


def transcript(messages):
    """Return the session as plain lines for the summary (tool traffic left out)."""
    lines = []
    for m in messages:
        content = m.get('content') or ''
        if not isinstance(content, str):  # a photo message
            continue
        if m['role'] == 'user' and content:
            lines.append(content if content.startswith('[Robot event]') else f'Person: {content}')
        elif m['role'] == 'assistant' and content:
            lines.append(f'Rainbow: {content}')
    return '\n'.join(lines)


def parse_summary(text):
    """Return (summary, facts) from the model's JSON reply; tolerant of extra text."""
    start, end = text.find('{'), text.rfind('}')
    try:
        data = json.loads(text[start:end + 1]) if start >= 0 else {}
    except ValueError:
        data = {}
    summary = str(data.get('summary') or '').strip()
    facts = [str(f).strip() for f in data.get('facts') or [] if str(f).strip()]
    return summary, facts


class Memory:

    def __init__(self, path, llm, model, embedder, recall_threshold=0.35, same_fact=0.9):
        """Open (or create) the database. embedder: async list[str] -> list[vector]."""
        self.db = sqlite3.connect(str(path))
        self.db.executescript(SCHEMA)
        self.llm, self.model, self.embed = llm, model, embedder
        self.recall_threshold, self.same_fact = recall_threshold, same_fact

    def close(self):
        self.db.close()

    # Writing

    async def store_session(self, messages, started, ended=None):
        """Summarise a finished session and keep its summary and new facts."""
        text = transcript(messages)
        if not any(line.startswith('Person:') for line in text.splitlines()):
            return None  # only robot events: nothing worth remembering
        response = await self.llm.chat.completions.create(
            model=self.model, temperature=0.2, max_tokens=400,
            messages=[{'role': 'system', 'content': SUMMARY_PROMPT},
                      {'role': 'user', 'content': text}])
        summary, facts = parse_summary(response.choices[0].message.content or '')
        if not summary:
            return None
        said = ' '.join(m['content'] for m in messages
                        if m['role'] == 'user' and isinstance(m.get('content'), str)
                        and not m['content'].startswith('['))
        facts = [f for f in facts if grounded_in(f, said)]
        vectors = await self.embed([summary] + facts)
        now = time.time()
        self.db.execute('INSERT INTO episodes (started, ended, summary, embedding) '
                        'VALUES (?, ?, ?, ?)', (started, ended or now, summary, _blob(vectors[0])))
        for fact, vector in zip(facts, vectors[1:]):
            self._remember_fact(fact, vector, now)
        self.db.commit()
        return summary, facts

    def _remember_fact(self, text, vector, now):
        v = np.asarray(_vector(_blob(vector)))
        best_id, best = None, -1.0
        for fact_id, blob in self.db.execute('SELECT id, embedding FROM facts'):
            sim = float(np.dot(v, _vector(blob)))
            if sim > best:
                best_id, best = fact_id, sim
        if best_id is not None and best >= self.same_fact:
            self.db.execute('UPDATE facts SET text = ?, updated = ?, embedding = ? WHERE id = ?',
                            (text, now, _blob(vector), best_id))
        else:
            self.db.execute('INSERT INTO facts (created, updated, text, embedding) '
                            'VALUES (?, ?, ?, ?)', (now, now, text, _blob(vector)))

    # Reading

    async def recall(self, text, facts=5, episodes=2):
        """Return the memories relevant to text, as lines for the system prompt."""
        if not text.strip():
            return []
        query = np.asarray((await self.embed([QUERY_PREFIX + text]))[0], dtype=np.float32)
        query /= np.linalg.norm(query) or 1.0
        lines = []
        for table, column, n, label in (('facts', 'text', facts, 'Fact'),
                                        ('episodes', 'summary', episodes, 'Earlier')):
            rows = self.db.execute(f'SELECT {column}, embedding FROM {table}').fetchall()
            scored = sorted(((float(np.dot(query, _vector(b))), t) for t, b in rows),
                            reverse=True)
            lines += [f'{label}: {t}' for s, t in scored[:n] if s >= self.recall_threshold]
        last = self.db.execute(
            'SELECT summary FROM episodes ORDER BY ended DESC LIMIT 1').fetchone()
        if last and f'Earlier: {last[0]}' not in lines:
            lines.append(f'Last time: {last[0]}')
        return lines

    def counts(self):
        return {t: self.db.execute(f'SELECT COUNT(*) FROM {t}').fetchone()[0]
                for t in ('episodes', 'facts')}


def llama_embedder(client, model):
    """Return an embedder backed by the embeddings llama-server (OpenAI API)."""
    async def embed(texts):
        response = await client.embeddings.create(model=model, input=texts)
        return [d.embedding for d in sorted(response.data, key=lambda d: d.index)]
    return embed
