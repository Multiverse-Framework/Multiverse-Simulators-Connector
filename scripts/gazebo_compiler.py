#!/usr/bin/env python3

import os
import shutil
import sys
import xml.etree.ElementTree as ET
from scipy.spatial.transform import Rotation as R
from typing import Dict, Any, Union, Optional

current_dir = os.path.dirname(__file__)
lib_dir = os.path.abspath(os.path.join(current_dir, '..', 'src'))
sys.path.insert(0, lib_dir)

from multiverse_simulator import MultiverseSimulatorCompiler, Robot, Object, multiverse_simulator_compiler_main

import numpy

class GazeboCompiler(MultiverseSimulatorCompiler):
    name: str = "gazebo"
    ext: str = "sdf"

    def __init__(self, args):
        super().__init__(args)

    def add_entity(self, entity: Union[Dict[str, Robot], Dict[str, Object]], world_element: ET.Element):
        for entity_name, entity_data in entity.items():
            include_element = ET.SubElement(world_element, "include")
            uri_element = ET.SubElement(include_element, "uri")
            uri_element.text = entity_data.path
            entity_tree = ET.parse(entity_data.path)
            entity_root = entity_tree.getroot()
            entity_model = entity_root.find("model")
            if entity_model is None:
                raise ValueError(f"Model element not found in the SDF file for {entity_name}")
            entity_root_name = entity_model.get("name", entity_name)
            pos = [0.0, 0.0, 0.0]
            quat = [1.0, 0.0, 0.0, 0.0]
            apply_data = entity_data.apply
            if "body" in apply_data and entity_root_name in apply_data["body"]:
                pos = apply_data["body"][entity_root_name].get("pos", [0.0, 0.0, 0.0])
                quat = apply_data["body"][entity_root_name].get("quat", [1.0, 0.0, 0.0, 0.0])
            euler = [*R.from_quat(quat, scalar_first=True).as_euler('xyz', degrees=False)]
            pose = " ".join([str(p) for p in pos + euler])
            pose_element = ET.SubElement(include_element, "pose")
            pose_element.text = pose

    def build_world(self,
                    robots: Dict[str, Robot],
                    objects: Dict[str, Object],
                    references: Optional[Dict[str, Dict[str, Any]]] = None,
                    multiverse_params: Optional[Dict[str, Dict]] = None):
        shutil.copy(self.world_path, self.save_file_path)
        tree = ET.parse(self.save_file_path)
        root = tree.getroot()
        world_element = root.find("world")
        if world_element is None:
            raise ValueError("World element not found in the SDF file")
        self.add_entity(robots, world_element)
        self.add_entity(objects, world_element)
        file_xml_string = ET.tostring(root, encoding='unicode', method='xml')
        with open(self.save_file_path, 'w') as f:
            f.write(file_xml_string)


if __name__ == "__main__":
    multiverse_simulator_compiler_main(GazeboCompiler)
