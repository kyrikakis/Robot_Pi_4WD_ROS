"""Tests for the places file (no ROS needed): python3 -m pytest src/rainbow_brain/test."""

import math
import os

import pytest
from rainbow_brain.places import angle_diff, normalize_name, PlaceStore


@pytest.fixture
def store(tmp_path):
    return PlaceStore(str(tmp_path), 'home_live', log=lambda m: None)


def test_normalize_name():
    assert normalize_name('  The   Kitchen ') == 'kitchen'
    assert normalize_name("Kid's room") == "kid's room"
    for bad in ('', '   ', 'x' * 41, 'kitchen!', '../etc'):
        with pytest.raises(ValueError):
            normalize_name(bad)


def test_angle_diff():
    assert angle_diff(0.1, -0.1) == pytest.approx(0.2)
    assert angle_diff(math.pi - 0.1, -math.pi + 0.1) == pytest.approx(-0.2)


def test_save_reload_and_file(store, tmp_path):
    store.save('Kitchen', 3.2, -1.1, 1.57, 'kitchen', 'by the fridge')
    store.save('sofa', 0.8, 2.4, 3.14, 'living room', '', source='robot')
    again = PlaceStore(str(tmp_path), 'home_live')
    assert [p.name for p in again.sorted_places()] == ['kitchen', 'sofa']
    assert again.get('the kitchen').description == 'by the fridge'
    assert again.get('sofa').confirmed is False
    assert again.get('kitchen').confirmed is True
    assert os.path.exists(os.path.join(str(tmp_path), 'places', 'home_live.yaml'))


def test_maps_are_separate(store, tmp_path):
    store.save('kitchen', 1, 1, 0)
    assert PlaceStore(str(tmp_path), 'other_map').sorted_places() == []


def test_edit_rename_confirm_delete(store):
    store.save('sofa', 0, 0, 0, source='robot')
    store.save('kitchen', 5, 0, 0)
    with pytest.raises(ValueError):
        store.edit('sofa', new_name='kitchen')
    p = store.edit('sofa', new_name='couch', room='living room', confirm=True)
    assert (p.name, p.room, p.confirmed) == ('couch', 'living room', True)
    with pytest.raises(ValueError):
        store.get('sofa')
    store.delete('couch')
    assert [p.name for p in store.sorted_places()] == ['kitchen']
    with pytest.raises(ValueError):
        store.delete('couch')


def test_broken_file_is_kept_aside(tmp_path):
    os.makedirs(tmp_path / 'places')
    (tmp_path / 'places' / 'home_live.yaml').write_text('places: [not, a, dict')
    s = PlaceStore(str(tmp_path), 'home_live', log=lambda m: None)
    assert s.sorted_places() == []
    assert any(n.startswith('home_live.yaml.bad-') for n in os.listdir(tmp_path / 'places'))


def test_nearest_and_facing(store):
    store.save('kitchen', 2.0, 0.0, 0.0, 'kitchen')
    store.save('sofa', 0.0, 3.0, 0.0, 'living room')
    near, d = store.nearest(1.5, 0.0, 1.5)
    assert near.name == 'kitchen' and d == pytest.approx(0.5)
    assert store.nearest(10, 10, 1.5)[0] is None
    assert store.facing(0.0, 0.0, math.pi / 2).name == 'sofa'
    assert store.facing(0.0, 0.0, 0.0).name == 'kitchen'
    assert store.facing(0.0, 0.0, math.pi) is None


def test_describe(store):
    store.save('kitchen', 2.0, 0.0, 0.0, 'kitchen')
    store.save('sofa', 0.0, 3.0, 0.0, 'living room', source='robot')
    sentence, near, ahead = store.describe(1.5, 0.0, -math.pi / 2, 1.5)
    assert sentence == "I'm in the kitchen, near the kitchen."
    sentence, _, _ = store.describe(0.0, 2.5, math.atan2(-2.5, 2.0), 1.5, 'roughly')
    assert sentence == ("I think I'm in the living room, near what I think is the sofa, "
                        'facing the kitchen.')
    sentence, _, _ = store.describe(9, 9, 0, 1.5, 'lost')
    assert sentence.startswith("I'm not sure where I am yet. My best guess: I'm not near")


def test_guess_from_a_pick_stays_unconfirmed(store):
    p = store.save('nook', 1, 1, 0, source='foxglove', confirmed=False)
    assert (p.source, p.confirmed) == ('foxglove', False)


def test_command_phrases_and_parsing():
    from rainbow_brain.commands import parse, phrases
    places = ['kitchen', 'kids room', 'home']
    p = phrases(places)
    assert 'go to the kids room' in p and 'stop' in p and 'come back' in p
    assert parse('stop', places) == {'action': 'stop'}
    assert parse('Come back', places) == {'action': 'go_to', 'place': 'home'}
    assert parse('go to the kids room', places) == {'action': 'go_to', 'place': 'kids room'}
    assert parse('go to kitchen', places) == {'action': 'go_to', 'place': 'kitchen'}
    assert parse('go to the garage', places) is None
    assert parse('', places) is None
