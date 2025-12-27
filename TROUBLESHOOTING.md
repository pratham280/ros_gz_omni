# Troubleshooting Guide - ROS 2 Gazebo Omnidirectional Drive

This document contains comprehensive troubleshooting solutions for common issues encountered when using the ros_gz_omni project.

## Table of Contents

1. [Common Issues & Solutions](#common-issues--solutions)
2. [Performance Optimization](#performance-optimization)

---

## Common Issues & Solutions

### Issue 1: "ros2 launch not found"

**Symptom**: 
```
Command 'ros2' not found
```

**Solution**:
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros_gz_ws/install/setup.bash
```

**Permanent fix**: Add to `~/.bashrc`:
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros_gz_ws/install/setup.bash
```

---

### Issue 2: "Cannot find package ros_gz_omni_bringup"

**Symptom**: 
```
Package 'ros_gz_omni_bringup' not found
```

**Solution**:
1. Rebuild workspace:
   ```bash
   cd ~/ros_gz_ws
   colcon build --packages-select ros_gz_omni_bringup
   source install/setup.bash
   ```

2. Verify package exists:
   ```bash
   ros2 pkg list | grep ros_gz_omni
   ```

---

### Issue 3: Gazebo fails to start with "plugin not found"

**Symptom**:
```
[ERR] [gz.plugin.Loader] Failed to load plugin [.../OmniDrive]
```

**Solution**:
1. Check plugin was compiled:
   ```bash
   ls ~/ros_gz_ws/install/lib/ros_gz_omni_gazebo/
   # Should contain libOmniDrive.so, libBasicSystem.so, libFullSystem.so
   ```

2. Rebuild gazebo packages:
   ```bash
   cd ~/ros_gz_ws
   colcon build --packages-select ros_gz_omni_gazebo
   source install/setup.bash
   ```

3. Set plugin path explicitly:
   ```bash
   export GZ_PLUGIN_PATH=$GZ_PLUGIN_PATH:~/ros_gz_ws/install/lib/ros_gz_omni_gazebo
   ros2 launch ros_gz_omni_bringup omni_drive.launch.py
   ```

---

### Issue 4: "Failed to load SDF file"

**Symptom**:
```
Error: Unable to find file [package://ros_gz_omni_description/models/omni_drive/model.sdf]
```

**Solution**:
1. Verify model file exists:
   ```bash
   find ~/ros_gz_ws -name "model.sdf" -type f
   ```

2. Rebuild description package:
   ```bash
   cd ~/ros_gz_ws
   colcon build --packages-select ros_gz_omni_description
   source install/setup.bash
   ```

3. Check package path:
   ```bash
   ros2 pkg prefix ros_gz_omni_description
   # Should return: /home/user/ros_gz_ws/install/ros_gz_omni_description
   ```

---

### Issue 5: RViz shows "No transform from [X] to [world]"

**Symptom**:
```
[ERROR] [rviz2]: Transform [omni_drive] does not exist
```

**Solution**:
1. Check robot_state_publisher is running:
   ```bash
   ros2 node list | grep robot_state
   ```

2. Verify /tf is being published:
   ```bash
   ros2 topic echo /tf --once
   ```

3. Restart robot_state_publisher:
   ```bash
   # Kill and restart launch
   ros2 launch ros_gz_omni_bringup omni_drive.launch.py
   ```

---

### Issue 6: Joystick not recognized

**Symptom**:
```
[joy-1] [ERROR] [1700000000.123456789]: Couldn't open joystick: /dev/input/js0
```

**Solution**:
1. Check joystick exists:
   ```bash
   ls -la /dev/input/js*
   ```

2. Grant permissions:
   ```bash
   sudo usermod -a -G input $USER
   # Log out and back in for changes to take effect
   ```

3. Connect different USB device:
   - May be on /dev/input/js1, /dev/input/js2, etc.
   - Check device number with: `jstest /dev/input/js*`

4. Specify device in launch:
   ```bash
   ros2 launch ros_gz_omni_bringup omni_drive.launch.py teleop:=true
   # Edit launch file to set correct device path
   ```

---

### Issue 7: "Gazebo crashes with segmentation fault"

**Symptom**:
```
Segmentation fault (core dumped)
```

**Solution**:
1. Check Gazebo version compatibility:
   ```bash
   gz sim --version
   # Should be 8.x for Harmonic
   ```

2. Update all Gazebo packages:
   ```bash
   sudo apt update && sudo apt upgrade gz-*
   ```

3. Rebuild with debug symbols:
   ```bash
   cd ~/ros_gz_ws
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
   ```

4. Run with gdb:
   ```bash
   gdb ros2 launch ros_gz_omni_bringup omni_drive.launch.py
   # At gdb prompt: run
   ```

---

### Issue 8: Gazebo Model Loading Difficulties

**Model mesh not displaying**:
```bash
# Check mesh file exists
find ~/ros_gz_ws -name "*.stl" | grep omni_drive

# Rebuild if missing:
cd ~/ros_gz_ws
colcon build --packages-select ros_gz_omni_description
source install/setup.bash
```

**SDF parse errors**:
```bash
# Validate SDF syntax:
gz sdf -p ~/ros_gz_ws/install/ros_gz_omni_description/share/ros_gz_omni_description/models/omni_drive/model.sdf

# View converted SDF (if from URDF):
gz sdf -p model.urdf > model_converted.sdf
```

**Collision geometry not matching visual**:
- Verify collision shapes in SDF match mesh geometry
- Check inertial properties for numerical accuracy
- Test with simplified collision geometry:
  ```xml
  <collision>
    <geometry>
      <cylinder>
        <radius>0.0525</radius>
        <length>0.01</length>
      </cylinder>
    </geometry>
  </collision>
  ```

---

### Issue 9: RViz Visualization Issues

**No robot displayed**:
1. Check `/robot_description` parameter exists:
   ```bash
   ros2 param get /robot_state_publisher robot_description | wc -c
   # Should be > 1000 characters
   ```

2. Reload URDF/SDF in RViz:
   - Select "RobotModel" display
   - Click "Refresh" button
   - Or restart RViz entirely

**Frames not showing**:
1. Enable TF display in RViz
2. Check all frames in dropdown are available
3. View transform tree:
   ```bash
   ros2 run tf2_tools view_frames
   pdf_viewer frames.pdf
   ```

**Markers not visible**:
- Verify marker topics in RViz configuration
- Check topic names match publisher output:
  ```bash
  ros2 topic list | grep marker
  ```

---

## Performance Optimization

**If simulation runs slow**:

1. Disable RViz:
   ```bash
   ros2 launch ros_gz_omni_bringup omni_drive.launch.py rviz:=false
   ```

2. Run Gazebo in headless mode:
   ```bash
   # Edit launch file: change '-s' argument only (no visual)
   gz sim world.sdf -s
   ```

3. Reduce simulation frequency:
   - Edit `omni_drive.sdf`: increase `<max_step_size>` (default 0.001s)

4. Disable unnecessary sensors/plugins in SDF

---

**Last Updated**: 2025-01-01  
**Gazebo Version**: Harmonic (gz-sim8)  
**ROS 2 Distribution**: Jazzy
