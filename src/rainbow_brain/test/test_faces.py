"""Tests for reading the Hailo's save_face answers: python3 -m pytest src/rainbow_brain/test."""

from rainbow_brain.faces import save_outcome


def test_the_three_already_known_cases_share_code_1():
    assert save_outcome(1, 'Similar embedding found with the same id. Aborted, '
                           'Consider calling with append=true') == 'same'
    assert save_outcome(1, 'Similar embedding found with different id. Aborted, '
                           'Consider calling with append=true') == 'other_name'
    assert save_outcome(1, 'The id exists with distant embedding(s). Aborted, '
                           'Consider calling with append=true') == 'distant'


def test_the_other_codes():
    assert save_outcome(0, 'Person added') == 'added'
    assert save_outcome(2, 'Error: Multiple faces found') == 'many'
    assert save_outcome(3, 'Error: No faces found') == 'none'
    assert save_outcome(4, 'Failed, see the logs for more details') == 'error'
