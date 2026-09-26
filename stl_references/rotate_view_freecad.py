import math
import os

import FreeCAD as App
import FreeCADGui as Gui

try:
    from PySide6 import QtCore
except ImportError:
    from PySide2 import QtCore


root = os.path.dirname(os.path.abspath(__file__))
doc_path = os.path.join(root, "AMR_STL_Trial.FCStd")
log_path = os.path.join(root, "rotate_view_freecad.log")


def log(message):
    with open(log_path, "a", encoding="utf-8") as handle:
        handle.write(message + "\n")


doc = App.openDocument(doc_path)
Gui.ActiveDocument = Gui.getDocument(doc.Name)
Gui.activateWorkbench("MeshWorkbench")
log("opened " + doc_path)

view = Gui.ActiveDocument.ActiveView
view.viewAxonometric()
view.fitAll()
log("fit all")

base_tilt = App.Rotation(App.Vector(1, 0, 0), -60)
step_count = 120
interval_ms = 40
state = {"i": 0}


def rotate_once():
    i = state["i"]
    angle = (360.0 * i) / step_count
    yaw = App.Rotation(App.Vector(0, 0, 1), angle)
    view.setCameraOrientation(yaw.multiply(base_tilt))
    view.fitAll()
    state["i"] += 1
    if state["i"] > step_count:
        timer.stop()
        view.viewAxonometric()
        view.fitAll()
        log("rotation complete")


timer = QtCore.QTimer()
timer.timeout.connect(rotate_once)
timer.start(interval_ms)
log("timer started")
