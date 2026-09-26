# AMR STL References

Downloaded on 2026-09-26 for the AMR / ROS 2 / Jetson learning project.

## BumbleBot

Source: `Build Autonomous Mobile Robot from Scratch using ROS`, O'Reilly Ch. 7.

- `bumblebot/Bumblebot_3d_Models.zip` is the original model archive.
- `bumblebot/Base Plate.stl` is the original extracted file name.
- `bumblebot/Base_Plate.stl` is the normalized copy for easier URDF mesh references.

Use `Base_Plate.stl` first to practice the STL to URDF workflow:

```xml
<link name="base_link">
  <visual>
    <geometry>
      <mesh filename="package://YOUR_PACKAGE/meshes/Base_Plate.stl" scale="0.001 0.001 0.001"/>
    </geometry>
  </visual>
</link>
```

Check scale before using it in ROS. STL files are often authored in millimeters, while URDF uses meters.

## Skycam Camera Mount

Source: `3D Printing Projects`, O'Reilly Ch. 8 Skycam.

These are reference parts for future cuVSLAM / camera bracket design:

- `skycam_camera_mount/Skycam-camera-front.stl`
- `skycam_camera_mount/Skycam-camera-back.stl`
- `skycam_camera_mount/Skycam-camera-pan.stl`
- `skycam_camera_mount/Skycam-camera-tilt.stl`
- `skycam_camera_mount/Skycam-pan-tilt-top.stl`

Use these as design references, not as final Jetson camera mounts. For cuVSLAM, the important constraints are rigid mounting, known camera pose, low vibration, and repeatable alignment.
