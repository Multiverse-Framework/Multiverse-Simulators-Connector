#!/usr/bin/env python3

import os
import sys
from typing import Dict, Any, Optional

import numpy
import shutil
import json
import yaml

current_dir = os.path.dirname(__file__)
lib_dir = os.path.abspath(os.path.join(current_dir, '..', 'src'))
sys.path.insert(0, lib_dir)

external_parent_dir = os.path.abspath(os.path.join(current_dir, '..', '..', '..', '..', '..', 'Multiverse-Parser'))
if os.name == 'nt':
    usd_dir = os.path.abspath(os.path.join(external_parent_dir, 'USD', 'windows', 'lib', 'python'))
    os.environ["PATH"] += f";{os.path.abspath(os.path.join(external_parent_dir, 'USD', 'windows', 'bin'))}"
    os.environ["PATH"] += f";{os.path.abspath(os.path.join(external_parent_dir, 'USD', 'windows', 'lib'))}"
    os.environ["PATH"] += f";{os.path.abspath(os.path.join(external_parent_dir, 'USD', 'windows', 'plugin', 'usd'))}"
else:
    usd_dir = os.path.abspath(os.path.join(external_parent_dir, 'USD', 'linux', 'lib', 'python'))
    os.environ["PATH"] += f":{os.path.abspath(os.path.join(external_parent_dir, 'USD', 'linux', 'lib'))}"
    os.environ["PATH"] += f":{os.path.abspath(os.path.join(external_parent_dir, 'USD', 'linux', 'plugin', 'usd'))}"
sys.path.insert(0, usd_dir)

from multiverse_simulator import MultiverseSimulatorCompiler, Robot, Object, multiverse_simulator_compiler_main


class IsaacSimCompiler(MultiverseSimulatorCompiler):
    name: str = "isaac_sim"
    ext: str = "usda"
    world_stage: Optional["Usd.Stage"] = None

    def __init__(self, args):
        super().__init__(args)

    def build_world(self,
                    robots: Dict[str, Robot],
                    objects: Dict[str, Object],
                    references: Optional[Dict[str, Dict[str, Any]]] = None,
                    multiverse_params: Optional[Dict[str, Dict]] = None):
        from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf  # Ask NVIDIA for this shitty importing style

        for entity in list(robots.values()) + list(objects.values()):
            file_ext = os.path.splitext(entity.path)[1]
            entity_usd_dir = os.path.dirname(entity.path)
            entity_usd_path = os.path.join(self.save_dir_path, entity.name + f"{file_ext}")
            shutil.copy(entity.path, entity_usd_path)
            entity.path = entity_usd_path

            if file_ext == ".usda":
                with open(entity_usd_path, "r") as f:
                    data = f.read()
                data = data.replace("@./", f"@{entity_usd_dir}/")
                with open(entity_usd_path, "w") as f:
                    f.write(data)
            entity_stage = Usd.Stage.Open(entity_usd_path)
            entity_editor = Usd.NamespaceEditor(entity_stage)

            if "body" in entity.apply:
                body_apply = entity.apply["body"]
                xform_prims = [prim for prim in entity_stage.Traverse() if prim.IsA(UsdGeom.Xform) and prim.GetName() in body_apply]
                bodies_applied = []
                for xform_prim in xform_prims:
                    if xform_prim.GetName() in bodies_applied:
                        continue
                    bodies_applied.append(xform_prim.GetName())
                    xform = UsdGeom.Xform(xform_prim)
                    pose = xform.GetLocalTransformation()
                    pos = pose.ExtractTranslation()
                    pos = [*pos]
                    quat = pose.ExtractRotationQuat()
                    quat = [*quat.GetImaginary(), quat.GetReal()]
                    xform.ClearXformOpOrder()
                    pos = body_apply[xform_prim.GetName()].get("pos", pos)
                    quat = body_apply[xform_prim.GetName()].get("quat", quat)

                    pos = numpy.asarray(pos, dtype=numpy.float64)
                    quat = numpy.asarray(quat, dtype=numpy.float64)
                    quat = quat / numpy.linalg.norm(quat)
                    mat = Gf.Matrix4d()
                    mat.SetTranslateOnly(Gf.Vec3d(*pos))
                    mat.SetRotateOnly(Gf.Quatd(quat[0], Gf.Vec3d(*quat[1:])))

                    xform.AddTransformOp().Set(mat)

            for joint_name, joint_value in entity.joint_state.items():
                for joint_prim in [prim for prim in entity_stage.TraverseAll() if
                                   prim.IsA(UsdPhysics.Joint) and prim.GetName() == joint_name]:
                    if joint_prim.HasAPI(UsdPhysics.DriveAPI):
                        drive_api = UsdPhysics.DriveAPI.Apply(joint_prim, "angular" if joint_prim.IsA(UsdPhysics.RevoluteJoint) else "linear")
                        drive_api.GetTargetPositionAttr().Set(numpy.rad2deg(joint_value) if joint_prim.IsA(UsdPhysics.RevoluteJoint) else joint_value)
                    else:
                        print(f"Joint {joint_name} does not have DriveAPI")

            entity_prim = entity_stage.GetDefaultPrim()
            if entity_prim.GetName() != entity.name:
                entity_editor.RenamePrim(entity_prim, entity.name)
                entity_editor.ApplyEdits()

            for joint_prim in [prim for prim in entity_stage.TraverseAll() if prim.IsA(UsdPhysics.Joint)]:
                joint_prim_name = entity.prefix.get("joint", "") + joint_prim.GetName() + entity.suffix.get("joint", "")
                if joint_prim.GetName() != joint_prim_name:
                    entity_editor.RenamePrim(joint_prim, joint_prim_name)
                    entity_editor.ApplyEdits()

            entity_stage.GetRootLayer().Save()

        robots_path = os.path.join(self.save_dir_path, os.path.basename(self.save_file_path).split(".")[0] + "_robots.usda")
        robots_stage = Usd.Stage.CreateNew(robots_path)
        robots_prim_path = Sdf.Path("/Robots")
        robots_xform = UsdGeom.Xform.Define(robots_stage, robots_prim_path)
        robots_prim = robots_xform.GetPrim()
        robots_stage.SetDefaultPrim(robots_prim)

        for robot in robots.values():
            robot_prim_path = robots_prim_path.AppendChild(robot.name)
            robot_xform = UsdGeom.Xform.Define(robots_stage, robot_prim_path)
            robot_prim = robot_xform.GetPrim()
            robot_stage = Usd.Stage.Open(robot.path)
            robot_prim.GetReferences().AddReference(robot_stage.GetRootLayer().identifier, robot_stage.GetDefaultPrim().GetPath())

        robots_stage.Flatten()
        robots_stage.Export(robots_path)

        print(f"Robots: {robots_path}")

        objects_path = os.path.join(self.save_dir_path, os.path.basename(self.save_file_path).split(".")[0] + "_objects.usda")
        objects_stage = Usd.Stage.CreateNew(objects_path)
        objects_prim_path = Sdf.Path("/Objects")
        objects_xform = UsdGeom.Xform.Define(objects_stage, objects_prim_path)
        objects_prim = objects_xform.GetPrim()
        objects_stage.SetDefaultPrim(objects_prim)

        for object in objects.values():
            object_prim_path = objects_prim_path.AppendChild(object.name)
            object_xform = UsdGeom.Xform.Define(objects_stage, object_prim_path)
            object_prim = object_xform.GetPrim()
            object_stage = Usd.Stage.Open(object.path)
            object_prim.GetReferences().AddReference(object_stage.GetRootLayer().identifier, object_stage.GetDefaultPrim().GetPath())

        objects_stage.Flatten()
        objects_stage.Export(objects_path)

        print(f"Objects: {objects_path}")

        file_ext = os.path.splitext(self.world_path)[1]
        if file_ext == ".usda":
            with open(self.save_file_path, "r") as f:
                data = f.read()
            world_usd_dir = os.path.dirname(self.world_path)
            data = data.replace("@./", f"@{world_usd_dir}/")
            with open(self.save_file_path, "w") as f:
                f.write(data)

        self.world_stage = Usd.Stage.Open(self.save_file_path)
        subplayer_paths = [robots_path, objects_path]
        self.world_stage.GetRootLayer().subLayerPaths = subplayer_paths
        self.world_stage.Export(self.save_file_path)

        if references is not None and references != {}:
            self.add_references(references)
            self.world_stage.Export(self.save_file_path)

        if multiverse_params is not None and multiverse_params != {}:
            self.world_stage = Usd.Stage.Open(self.save_file_path)
            customLayerData = self.world_stage.GetRootLayer().customLayerData
            customLayerData["multiverse_connector"] = {}
            customLayerData["multiverse_connector"]["host"] = multiverse_params.get("host", "tcp://127.0.0.1")
            customLayerData["multiverse_connector"]["server_port"] = multiverse_params.get("server_port", "7000")
            customLayerData["multiverse_connector"]["client_port"] = multiverse_params.get("client_port", "8000")
            customLayerData["multiverse_connector"]["world_name"] = multiverse_params.get("world_name", "world")
            customLayerData["multiverse_connector"]["simulation_name"] = multiverse_params.get("simulation_name", "isaac_sim")
            for send_receive in ["send", "receive"]:
                customLayerData["multiverse_connector"][send_receive] = multiverse_params.get(send_receive, {})
                for key, values in customLayerData["multiverse_connector"][send_receive].items():
                    customLayerData["multiverse_connector"][send_receive][key] = json.dumps(values)
            self.world_stage.GetRootLayer().customLayerData = customLayerData
            self.world_stage.GetRootLayer().Save()

            current_dir = os.path.dirname(os.path.abspath(__file__))
            tmp_dir = os.path.join(current_dir, "..", "src", "isaac_sim_connector", "extensions", "multiverse_connector", "tmp")
            tmp_path = os.path.join(tmp_dir, "multiverse_connector.yaml")

            def to_list_if_json(s):
                try:
                    v = json.loads(s)
                    return v if isinstance(v, list) else s
                except Exception:
                    return s

            multiverse_params["send"] = {k: to_list_if_json(v) for k, v in multiverse_params.get("send", {}).items()}
            multiverse_params["receive"] = {k: to_list_if_json(v) for k, v in multiverse_params.get("receive", {}).items()}

            with open(tmp_path, "w") as f:
                yaml.dump(multiverse_params, f)

    def add_references(self, references: Dict[str, Dict[str, Any]]):
        from pxr import UsdGeom, UsdPhysics # Ask NVIDIA for this shitty importing style

        xform_cache = UsdGeom.XformCache()
        references_xform = UsdGeom.Xform.Define(self.world_stage, "/References")
        reference_prim = references_xform.GetPrim()
        references_path = reference_prim.GetPath()
        for reference_name, reference in references.items():
            body_ref_name = reference.get("body1", None)
            if body_ref_name is None:
                raise ValueError("Reference must have a body1")
            body_name = reference.get("body2", None)
            if body_name is None:
                raise ValueError("Reference must have a body2")
            body_prim = None
            for prim in self.world_stage.Traverse():
                if prim.GetName() == body_name:
                    body_prim = prim
            assert body_prim is not None, f"Body {body_name} not found"
            mat, _ = xform_cache.ComputeRelativeTransform(body_prim, reference_prim)
            body_ref_path = references_path.AppendChild(body_ref_name)
            body_ref_xform = UsdGeom.Xform.Define(self.world_stage, body_ref_path)
            body_ref_xform.AddTransformOp().Set(mat)
            body_ref_prim = body_ref_xform.GetPrim()
            rigid_body_api = UsdPhysics.RigidBodyAPI.Apply(body_ref_prim)
            fixed_joint_path = body_ref_prim.GetPath().AppendChild("fixed_joint")
            fixed_joint = UsdPhysics.FixedJoint.Define(self.world_stage, fixed_joint_path)
            fixed_joint.GetBody0Rel().SetTargets([body_ref_prim.GetPath()])
            fixed_joint.GetBody1Rel().SetTargets([body_prim.GetPath()])

if __name__ == "__main__":
    multiverse_simulator_compiler_main(IsaacSimCompiler)
