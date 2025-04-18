import pytest
import numpy as np
import os
from examples import get_all_examples

def test_get_all_examples():
    # Test getting all examples
    examples = get_all_examples()
    assert examples is not None
    assert len(examples) > 0
    
    # Verify example format
    for name, example in examples.items():
        assert 'link_lengths' in example
        assert 'joint_angles' in example
        assert 'obstacles' in example
        assert 'start' in example
        assert 'goal' in example
        
        # Verify link_lengths and joint_angles have same length
        assert len(example['link_lengths']) == len(example['joint_angles'])
        
        # Verify start and goal are tuples
        assert isinstance(example['start'], tuple)
        assert isinstance(example['goal'], tuple)
        
        # Verify start and goal have same length
        assert len(example['start']) == len(example['goal'])
        
        # Verify start and goal length matches link_lengths length
        assert len(example['start']) == len(example['link_lengths'])

def test_get_all_examples_file_not_found():
    # Test file not found case
    with pytest.raises(FileNotFoundError):
        get_all_examples("non_existent_file.yml") 