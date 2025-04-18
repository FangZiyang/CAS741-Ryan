import pytest
import numpy as np
import os
from examples import get_all_examples

def test_get_all_examples():
    # 测试获取所有示例
    examples = get_all_examples()
    assert examples is not None
    assert len(examples) > 0
    
    # 验证示例格式
    for name, example in examples.items():
        assert 'link_lengths' in example
        assert 'joint_angles' in example
        assert 'obstacles' in example
        assert 'start' in example
        assert 'goal' in example
        
        # 验证link_lengths和joint_angles长度一致
        assert len(example['link_lengths']) == len(example['joint_angles'])
        
        # 验证start和goal是元组
        assert isinstance(example['start'], tuple)
        assert isinstance(example['goal'], tuple)
        
        # 验证start和goal长度一致
        assert len(example['start']) == len(example['goal'])
        
        # 验证start和goal长度与link_lengths长度一致
        assert len(example['start']) == len(example['link_lengths'])

def test_get_all_examples_file_not_found():
    # 测试文件不存在的情况
    with pytest.raises(FileNotFoundError):
        get_all_examples("non_existent_file.yml") 