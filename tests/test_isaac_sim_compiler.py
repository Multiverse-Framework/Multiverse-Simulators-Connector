#!/usr/bin/env python3

import argparse
import os
import sys
import unittest
from unittest.mock import patch
import tracemalloc

from multiverse_simulator import multiverse_simulator_compiler_main
current_dir = os.path.dirname(__file__)
scripts_dir = os.path.abspath(os.path.join(current_dir, '..', 'scripts'))
sys.path.insert(0, scripts_dir)
from isaac_sim_compiler import IsaacSimCompiler


resources_path = os.path.join(os.path.dirname(__file__), "..", "resources")
save_dir_path = os.path.join(resources_path, "saved")


class MultiverseSimulatorCompilerTestCase(unittest.TestCase):
    @patch('argparse.ArgumentParser.parse_args',
           return_value=argparse.Namespace(name="multiverse_simulation",
                                           world_path="",
                                           robots="{}",
                                           objects="{}",
                                           references="{}",
                                           save_dir_path=save_dir_path,
                                           multiverse_params=None))
    def test_multiverse_simulator_compiler_main(self, _):
        self.assertRaises(AssertionError, multiverse_simulator_compiler_main)


class IsaacSimCompilerTestCase(MultiverseSimulatorCompilerTestCase):
    world_path = os.path.join(resources_path, "usd/floor/floor.usda")
    shadow_hand = {
        "left_hand": {
            "path": os.path.join(resources_path, "usd/shadow_hand/left_hand.usda"),
            "apply": {
                "body": {
                    "lh_palm": {
                        "pos": [0.24126255555095805, 0.1516456014816617, 0.8878448256201764],
                        "quat": [1.0, 0.0, 1.0, 0.0],
                    }
                }
            },
            "prefix": {
                "joint": "isaac_sim_",
            }
        },
        "right_hand": {
            "path": os.path.join(resources_path, "usd/shadow_hand/right_hand.usda"),
            "apply": {
                "body": {
                    "rh_palm": {
                        "pos": [0.24126255555095805, -0.1516456014816617, 0.8878448256201764],
                        "quat": [0.0, 1.0, 0.0, 1.0],
                    }
                }
            },
            "prefix": {
                "joint": "isaac_sim_",
            }
        },
    }
    multiverse_params = {
        "host": "tcp://127.0.0.1",
        "server_port": "7000",
        "client_port": "8000",
        "world_name": "world",
        "simulation_name": "mujoco_simulation_3",
        "send": {
            "isaac_sim_lh_palm": ["position", "quaternion", "linear_velocity", "angular_velocity"],
            "joint": ["joint_angular_position"]
        },
        "receive": {
            "isaac_sim_lh_palm": ["force", "torque"],
        }
    }
    references = {
        "reference_1": {
            "body1": "isaac_sim_lh_palm",
            "body2": "lh_palm",
        },
        "reference_2": {
            "body1": "isaac_sim_rh_palm",
            "body2": "rh_palm",
        }
    }

    @classmethod
    def setUpClass(cls):
        tracemalloc.start()
        if not os.path.exists(resources_path):
            print(f"Resource path {resources_path} does not exist, creating one...")
            os.makedirs(resources_path)
            import requests
            import zipfile
            import io
            from tqdm import tqdm
            file_name = "resources.zip"
            url = f"https://nc.uni-bremen.de/index.php/s/ScjCdWZ4cLn9ZZc/download/{file_name}"
            response = requests.get(url, stream=True)
            response.raise_for_status()  # raise exception if download failed
            total_size = int(response.headers.get('content-length', 0))
            buffer = io.BytesIO()
            with tqdm(
                    desc="Downloading resources...",
                    total=total_size,
                    unit='B',
                    unit_scale=True,
                    unit_divisor=1024,
            ) as bar:
                for chunk in response.iter_content(chunk_size=1024):
                    if chunk:
                        buffer.write(chunk)
                        bar.update(len(chunk))
            buffer.seek(0)
            with zipfile.ZipFile(buffer) as zip_ref:
                zip_ref.extractall(resources_path)
            print(f"Downloaded resources in {resources_path}")

    @classmethod
    def tearDownClass(cls):
        tracemalloc.stop()

    @patch('argparse.ArgumentParser.parse_args',
           return_value=argparse.Namespace(name="isaac_sim_simulation_1",
                                           world_path=world_path,
                                           robots="{}",
                                           objects="{}",
                                           references="{}",
                                           save_dir_path=save_dir_path,
                                           multiverse_params=None))
    def test_mujoco_compiler_world(self, _):
        multiverse_simulator_compiler_main(Compiler=IsaacSimCompiler)
        usda_file_path = os.path.join(save_dir_path, "isaac_sim_simulation_1.usda")
        self.assertTrue(os.path.exists(usda_file_path))

    @patch('argparse.ArgumentParser.parse_args',
           return_value=argparse.Namespace(name="isaac_sim_simulation_2",
                                           world_path=world_path,
                                           robots=str(shadow_hand),
                                           objects="{}",
                                           references="{}",
                                           save_dir_path=save_dir_path,
                                           multiverse_params=None))
    def test_mujoco_compiler_world_with_robot(self, _):
        multiverse_simulator_compiler_main(Compiler=IsaacSimCompiler)
        usda_file_path = os.path.join(save_dir_path, "isaac_sim_simulation_2.usda")
        self.assertTrue(os.path.exists(usda_file_path))

    @patch('argparse.ArgumentParser.parse_args',
           return_value=argparse.Namespace(name="isaac_sim_simulation_3",
                                           world_path=world_path,
                                           robots=str(shadow_hand),
                                           objects="{}",
                                           references="{}",
                                           save_dir_path=save_dir_path,
                                           multiverse_params=str(multiverse_params)))
    def test_mujoco_compiler_world_with_robots(self, _):
        multiverse_simulator_compiler_main(Compiler=IsaacSimCompiler)
        usda_file_path = os.path.join(save_dir_path, "isaac_sim_simulation_3.usda")
        self.assertTrue(os.path.exists(usda_file_path))

    @patch('argparse.ArgumentParser.parse_args',
           return_value=argparse.Namespace(name="isaac_sim_simulation_4",
                                           world_path=world_path,
                                           robots=str(shadow_hand),
                                           objects="{}",
                                           references=str(references),
                                           save_dir_path=save_dir_path,
                                           multiverse_params=str(multiverse_params)))
    def test_mujoco_compiler_world_with_robots_and_refs_and_multiverse(self, _):
        multiverse_simulator_compiler_main(Compiler=IsaacSimCompiler)
        xml_file_path = os.path.join(save_dir_path, "isaac_sim_simulation_4.usda")
        self.assertTrue(os.path.exists(xml_file_path))


if __name__ == "__main__":
    unittest.main()