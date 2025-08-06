#!/usr/bin/env python3

import os
import shutil
import sys
import xml.etree.ElementTree as ET
from scipy.spatial.transform import Rotation as R
from typing import Dict, Any, Union, Optional

current_dir = os.path.dirname(__file__)
lib_dir = os.path.abspath(os.path.join(current_dir, "..", "src"))
sys.path.insert(0, lib_dir)

from multiverse_simulator import (
    MultiverseSimulatorCompiler,
    Robot,
    Object,
    multiverse_simulator_compiler_main,
)

class GazeboCompiler(MultiverseSimulatorCompiler):
    name: str = "gazebo"
    ext: str = "sdf"

    def __init__(self, args):
        super().__init__(args)

    def add_entity(
        self,
        entity: Union[Dict[str, Robot], Dict[str, Object]],
        world_element: ET.Element,
    ):
        for entity_name, entity_data in entity.items():
            print(f"Adding entity: {entity_name}")
            entity_file_name = os.path.basename(entity_data.path)
            entity_file_path = os.path.join(self.save_dir_path, entity_file_name)
            shutil.copy(entity_data.path, entity_file_path)
            include_element = ET.SubElement(world_element, "include")
            uri_element = ET.SubElement(include_element, "uri")
            uri_element.text = entity_file_path
            entity_tree = ET.parse(entity_file_path)
            entity_root = entity_tree.getroot()
            for uri_element in entity_root.findall(".//uri"):
                if uri_element.text is None:
                    continue
                if uri_element.text.startswith("file://"):
                    uri_element.text = uri_element.text[len("file://") :]
                if not os.path.isabs(uri_element.text):
                    entity_file_abspath = os.path.join(
                        os.path.dirname(entity_data.path), uri_element.text
                    )
                    if os.path.exists(entity_file_abspath):
                        uri_element.text = entity_file_abspath
            entity_model = entity_root.find("model")
            apply_data = entity_data.apply
            if "body" in apply_data:
                for body_name, body_data in apply_data["body"].items():
                    if isinstance(body_data, dict):
                        link_element = entity_root.find(f".//link[@name='{body_name}']")
                        if link_element is None:
                            continue
                        for attribute_name, attribute_value in body_data.items():
                            if attribute_name in ["pos", "quat"]:
                                continue
                            attribute_element = ET.SubElement(
                                link_element, attribute_name
                            )
                            attribute_element.text = (
                                attribute_value
                                if isinstance(attribute_value, str)
                                else (
                                    " ".join([*attribute_value])
                                    if isinstance(attribute_value, (list, tuple))
                                    else str(attribute_value)
                                )
                            )
                    elif isinstance(body_data, str):
                        for link_element in entity_root.findall(".//link"):
                            pose_element = ET.SubElement(link_element, body_name)
                            pose_element.text = body_data
                entity_root_name = None
                if entity_model is not None:
                    entity_root_name = entity_model.get("name", entity_name)
                    pos = [0.0, 0.0, 0.0]
                    quat = [1.0, 0.0, 0.0, 0.0]
                    if entity_root_name in apply_data["body"]:
                        pos = apply_data["body"][entity_root_name].get(
                            "pos", [0.0, 0.0, 0.0]
                        )
                        quat = apply_data["body"][entity_root_name].get(
                            "quat", [1.0, 0.0, 0.0, 0.0]
                        )
                    euler = [
                        *R.from_quat(quat, scalar_first=True).as_euler(
                            "xyz", degrees=False
                        )
                    ]
                    pose = " ".join([str(p) for p in pos + euler])
                    pose_element = ET.SubElement(include_element, "pose")
                    pose_element.text = pose
            ET.indent(entity_root)
            entity_tree.write(entity_file_path, encoding="utf-8", xml_declaration=True)

    def build_world(
        self,
        robots: Dict[str, Robot],
        objects: Dict[str, Object],
        references: Optional[Dict[str, Dict[str, Any]]] = None,
        multiverse_params: Optional[Dict[str, Dict]] = None,
    ):
        shutil.copy(self.world_path, self.save_file_path)
        tree = ET.parse(self.save_file_path)
        root = tree.getroot()
        world_element = root.find("world")
        if world_element is None:
            raise ValueError("World element not found in the SDF file")
        self.add_entity(robots, world_element)
        self.add_entity(objects, world_element)
        file_xml_string = ET.tostring(root, encoding="unicode", method="xml")
        with open(self.save_file_path, "w") as f:
            f.write(file_xml_string)

        if multiverse_params != {} and multiverse_params is not None:
            print(multiverse_params)
            tree = ET.parse(self.save_file_path)
            plugin = f"""
                <plugin filename="MultiverseConnector">
                    <host>{multiverse_params['host']}</host>
                    <server_port>{multiverse_params['server_port']}</server_port>
                    <client_port>{multiverse_params['client_port']}</client_port>
                    <world_name>{multiverse_params['world_name']}</world_name>
                    <simulation_name>{multiverse_params['simulation_name']}</simulation_name>
                    <send>{multiverse_params['send']}</send>
                    <receive>{multiverse_params['receive']}</receive>
                </plugin>
                """
            root = tree.getroot()
            world_element = root.find("world")
            if world_element is None:
                raise ValueError("World element not found in the SDF file")
            world_element.append(ET.fromstring(plugin))
            ET.indent(root)
            file_xml_string = ET.tostring(root, encoding="unicode", method="xml")
            with open(self.save_file_path, "w") as f:
                f.write(file_xml_string)


if __name__ == "__main__":
    multiverse_simulator_compiler_main(GazeboCompiler)
