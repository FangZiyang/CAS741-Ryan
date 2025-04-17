import yaml
import numpy as np
from pathlib import Path
from src.planner.joint_limits import JointLimits


def get_all_examples(yaml_file="examples.yml"):
    path = Path(yaml_file)
    if not path.exists():
        raise FileNotFoundError(f"YAML file {yaml_file} not found.")

    with open(path, 'r') as file:
        raw_data = yaml.safe_load(file)

    examples = {}
    for name, info in raw_data.items():
        # Convert list of joint limit pairs to JointLimits object, if exists
        joint_limits = None
        if "joint_limits" in info:
            joint_limits = JointLimits(info["joint_limits"])

        examples[name] = {
            "link_lengths": info["link_lengths"],
            "joint_angles": info.get("joint_angles", [0] * len(info["link_lengths"])),
            "joint_limits": joint_limits,
            "obstacles": info["obstacles"],
            "start": tuple(info["start"]),
            "goal": tuple(info["goal"]),
        }

    return examples
