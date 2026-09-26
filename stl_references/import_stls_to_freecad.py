import os

import FreeCAD
import Mesh


root = os.path.dirname(os.path.abspath(__file__))
files = [
    os.path.join(root, "bumblebot", "Base_Plate.stl"),
    os.path.join(root, "skycam_camera_mount", "Skycam-camera-front.stl"),
    os.path.join(root, "skycam_camera_mount", "Skycam-camera-back.stl"),
    os.path.join(root, "skycam_camera_mount", "Skycam-camera-pan.stl"),
    os.path.join(root, "skycam_camera_mount", "Skycam-camera-tilt.stl"),
    os.path.join(root, "skycam_camera_mount", "Skycam-pan-tilt-top.stl"),
]

doc = FreeCAD.newDocument("AMR_STL_Trial")

for path in files:
    mesh = Mesh.Mesh(path)
    name = os.path.splitext(os.path.basename(path))[0].replace("-", "_").replace(" ", "_")
    obj = doc.addObject("Mesh::Feature", name)
    obj.Mesh = mesh
    obj.Label = os.path.basename(path)

doc.recompute()
doc.saveAs(os.path.join(root, "AMR_STL_Trial.FCStd"))
