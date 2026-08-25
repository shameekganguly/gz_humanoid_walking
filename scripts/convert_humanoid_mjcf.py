#!/usr/bin/env python3
"""
Converts the JVRC1 MJCF model to a Gazebo SDFormat model and configures
meshes, collision friction parameters, metadata, and custom visual materials:
- Light gray limbs
- Orange hands
- Orange feet
- Orange torso
- White head
"""

import os
import sys
import xml.etree.ElementTree as ET

# Ensure sdformat_mjcf is in path
WORKSPACE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))
MJCF_CONVERTER_SRC = os.path.join(WORKSPACE_ROOT, "src/gz-mujoco/sdformat_mjcf/src")
if MJCF_CONVERTER_SRC not in sys.path:
    sys.path.insert(0, MJCF_CONVERTER_SRC)

from sdformat_mjcf.mjcf_to_sdformat.mjcf_to_sdformat import mjcf_file_to_sdformat


def get_body_color(link_name: str):
    """
    Returns (diffuse, ambient, specular) RGBA strings based on body part.
    - Torso: Orange
    - Head: White
    - Hands: Orange
    - Feet: Orange
    - Limbs (thighs, shins, shoulders, elbows): Light Gray
    """
    name = link_name.upper()

    # Pelvis (Dark Gray)
    if "PELVIS" in name:
        diffuse = "0.25 0.25 0.25 1.0"
        ambient = "0.20 0.20 0.20 1.0"
        specular = "0.25 0.25 0.25 1.0"
    # Torso (Waist / Chest - Orange)
    elif "WAIST" in name:
        diffuse = "0.95 0.45 0.05 1.0"
        ambient = "0.85 0.40 0.05 1.0"
        specular = "0.3 0.3 0.3 1.0"
    # Head (Neck)
    elif "NECK" in name:
        diffuse = "0.95 0.95 0.95 1.0"
        ambient = "0.90 0.90 0.90 1.0"
        specular = "0.4 0.4 0.4 1.0"
    # Hands (Wrist, Thumb, Index, Little fingers)
    elif any(k in name for k in ["WRIST", "THUMB", "INDEX", "LITTLE"]):
        diffuse = "0.95 0.45 0.05 1.0"
        ambient = "0.85 0.40 0.05 1.0"
        specular = "0.3 0.3 0.3 1.0"
    # Feet (Ankles)
    elif "ANKLE" in name:
        diffuse = "0.95 0.45 0.05 1.0"
        ambient = "0.85 0.40 0.05 1.0"
        specular = "0.3 0.3 0.3 1.0"
    # Limbs (Hip, Knee, Shoulder, Elbow)
    else:
        diffuse = "0.75 0.75 0.75 1.0"
        ambient = "0.70 0.70 0.70 1.0"
        specular = "0.2 0.2 0.2 1.0"

    return diffuse, ambient, specular


def convert_jvrc1():
    pkg_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    mjcf_path = os.path.join(WORKSPACE_ROOT, "src/jvrc_mj_description/xml/jvrc1.xml")
    models_dir = os.path.join(pkg_dir, "models/jvrc1")
    meshes_dir = os.path.join(models_dir, "meshes")
    os.makedirs(models_dir, exist_ok=True)

    temp_sdf_path = os.path.join(models_dir, "world_converted.sdf")
    model_sdf_path = os.path.join(models_dir, "model.sdf")
    model_config_path = os.path.join(models_dir, "model.config")

    print(f"Converting MJCF from {mjcf_path} to {temp_sdf_path}...")
    mjcf_file_to_sdformat(mjcf_path, temp_sdf_path, export_world_plugins=False)

    print("Extracting and styling humanoid model SDF...")
    tree = ET.parse(temp_sdf_path)
    root = tree.getroot()

    world = root.find("world")
    if world is None:
        raise RuntimeError("No <world> element found in converted SDF")

    humanoid_model = None
    for model in world.findall("model"):
        if model.get("name") == "model_for_PELVIS_S":
            humanoid_model = model
            break

    if humanoid_model is None:
        for model in world.findall("model"):
            if model.get("name") != "static":
                humanoid_model = model
                break

    if humanoid_model is None:
        raise RuntimeError("Failed to find humanoid model in converted SDF")

    # Rename model to jvrc1
    humanoid_model.set("name", "jvrc1")

    # Available obj meshes in meshes directory
    available_objs = set(
        f for f in os.listdir(meshes_dir) if f.endswith(".obj")
    ) if os.path.exists(meshes_dir) else set()

    for link in humanoid_model.findall("link"):
        link_name = link.get("name", "")
        diffuse, ambient, specular = get_body_color(link_name)

        # Process Visual Elements
        for visual in link.findall("visual"):
            geometry = visual.find("geometry")
            if geometry is not None:
                mesh = geometry.find("mesh")
                if mesh is not None:
                    uri = mesh.find("uri")
                    if uri is not None and uri.text:
                        mesh_filename = os.path.basename(uri.text)
                        base_name = os.path.splitext(mesh_filename)[0]

                        # Prefer .obj visual mesh if present
                        obj_name = base_name + ".obj"
                        if obj_name in available_objs:
                            uri.text = f"model://jvrc1/meshes/{obj_name}"
                        else:
                            uri.text = f"model://jvrc1/meshes/{mesh_filename}"

            # Apply stylized material
            material = visual.find("material")
            if material is None:
                material = ET.SubElement(visual, "material")
            else:
                material.clear()

            mat_amb = ET.SubElement(material, "ambient")
            mat_amb.text = ambient
            mat_diff = ET.SubElement(material, "diffuse")
            mat_diff.text = diffuse
            mat_spec = ET.SubElement(material, "specular")
            mat_spec.text = specular

            pbr = ET.SubElement(material, "pbr")
            metal = ET.SubElement(pbr, "metal")
            rough = ET.SubElement(metal, "roughness")
            rough.text = "0.4"
            met = ET.SubElement(metal, "metalness")
            met.text = "0.1"

        # Process Collision Elements
        for collision in link.findall("collision"):
            geometry = collision.find("geometry")
            if geometry is not None:
                mesh = geometry.find("mesh")
                if mesh is not None:
                    uri = mesh.find("uri")
                    if uri is not None and uri.text:
                        mesh_filename = os.path.basename(uri.text)
                        uri.text = f"model://jvrc1/meshes/{mesh_filename}"

            # Apply high friction on feet
            if any(f in link_name.upper() for f in ["ANKLE"]):
                surface = collision.find("surface")
                if surface is None:
                    surface = ET.SubElement(collision, "surface")
                friction = surface.find("friction")
                if friction is None:
                    friction = ET.SubElement(surface, "friction")
                ode = friction.find("ode")
                if ode is None:
                    ode = ET.SubElement(friction, "ode")
                mu = ode.find("mu")
                if mu is None:
                    mu = ET.SubElement(ode, "mu")
                mu.text = "1.0"
                mu2 = ode.find("mu2")
                if mu2 is None:
                    mu2 = ET.SubElement(ode, "mu2")
                mu2.text = "1.0"

    # Create root model SDF
    new_root = ET.Element("sdf", version="1.8")
    new_root.append(humanoid_model)

    new_tree = ET.ElementTree(new_root)
    ET.indent(new_tree, space="  ", level=0)
    new_tree.write(model_sdf_path, encoding="utf-8", xml_declaration=True)
    print(f"Saved styled humanoid model SDF to {model_sdf_path}")

    # Create model.config
    model_config_content = """<?xml version="1.0"?>
<model>
  <name>jvrc1</name>
  <version>1.0</version>
  <sdf version="1.8">model.sdf</sdf>
  <author>
    <name>ISRI-AIST / Open Robotics</name>
  </author>
  <description>
    JVRC-1 humanoid robot model converted from MJCF with custom color styling (Orange torso/hands/feet, white head, light gray limbs).
  </description>
</model>
"""
    with open(model_config_path, "w") as f:
        f.write(model_config_content)
    print(f"Saved model.config to {model_config_path}")


if __name__ == "__main__":
    convert_jvrc1()
