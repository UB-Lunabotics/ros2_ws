# ROS 2 Workspace Context

This file contains the structure and contents of the ROS 2 workspace.

## File: ./get_context.py
```python
import os

def collect_workspace_context(output_file='ros2_workspace_context.md'):
    # Define directories to ignore to keep the context clean
    ignore_dirs = {'.git', 'install', 'build', 'log', '__pycache__', '.pytest_cache'}
    # Define file extensions we care about for code context
    valid_extensions = {'.py', '.urdf', '.yaml', '.xml', '.launch.py', '.cpp', '.hpp', 'txt'}

    with open(output_file, 'w', encoding='utf-8') as f:
        f.write("# ROS 2 Workspace Context\n\n")
        f.write("This file contains the structure and contents of the ROS 2 workspace.\n\n")

        for root, dirs, files in os.walk('.'):
            # Filter out ignored directories
            dirs[:] = [d for d in dirs if d not in ignore_dirs]

            for file in files:
                if any(file.endswith(ext) for ext in valid_extensions):
                    file_path = os.path.join(root, file)
                    f.write(f"## File: {file_path}\n")
                    f.write("```")
                    
                    # Add language hint for markdown
                    ext = os.path.splitext(file)[1]
                    if ext == '.py': f.write('python')
                    elif ext == '.yaml': f.write('yaml')
                    elif ext == '.xml' or ext == '.urdf': f.write('xml')
                    
                    f.write("\n")
                    
                    try:
                        with open(file_path, 'r', encoding='utf-8') as content_file:
                            f.write(content_file.read())
                    except Exception as e:
                        f.write(f"Error reading file: {e}")
                    
                    f.write("\n```\n\n")

    print(f"Successfully generated {output_file}")

if __name__ == '__main__':
    collect_workspace_context()
```

## File: ./src/rover_autonomy/setup.py
```python
import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rover_autonomy'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
    (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wade',
    maintainer_email='mehrasoham51@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)

```

## File: ./src/rover_autonomy/package.xml
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>rover_autonomy</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="mehrasoham51@gmail.com">wade</maintainer>
  <license>TODO: License declaration</license>

  <depend>rclpy</depend>
  <depend>nav2_simple_commander</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>

```

## File: ./src/rover_autonomy/config/ros2_controllers.yaml
```yaml
controller_manager:
  ros__parameters:
    update_rate: 100
    use_sim_time: true

    drive_controller:
      type: diff_drive_controller/DiffDriveController

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

drive_controller:
  ros__parameters:
    left_wheel_names: ["wheel_fl_joint", "wheel_rl_joint"]
    right_wheel_names: ["wheel_fr_joint", "wheel_rr_joint"]
    wheel_separation: 0.50
    wheel_radius: 0.15
```

## File: ./src/rover_autonomy/test/test_pep257.py
```python
# Copyright 2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_pep257.main import main
import pytest


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    rc = main(argv=['.', 'test'])
    assert rc == 0, 'Found code style errors / warnings'

```

## File: ./src/rover_autonomy/test/test_copyright.py
```python
# Copyright 2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_copyright.main import main
import pytest


# Remove the `skip` decorator once the source file(s) have a copyright header
@pytest.mark.skip(reason='No copyright header has been placed in the generated source file.')
@pytest.mark.copyright
@pytest.mark.linter
def test_copyright():
    rc = main(argv=['.', 'test'])
    assert rc == 0, 'Found errors'

```

## File: ./src/rover_autonomy/test/test_flake8.py
```python
# Copyright 2017 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_flake8.main import main_with_errors
import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    rc, errors = main_with_errors(argv=[])
    assert rc == 0, \
        'Found %d code style errors / warnings:\n' % len(errors) + \
        '\n'.join(errors)

```

## File: ./src/rover_autonomy/launch/launch_robot.launch.py
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'rover_autonomy'

    # Path to your URDF
    urdf_path = os.path.join(get_package_share_directory(package_name), 'urdf', 'assembled_robot.urdf')
    with open(urdf_path, 'r') as infp:
        robot_description_config = infp.read()

    # 1. Robot State Publisher (Publishes the URDF)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config, 'use_sim_time': True}]
    )

    # 2. Gazebo (The World)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    # 3. Spawn the Robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'assembled_rover'],
        output='screen'
    )

    return LaunchDescription([
        node_robot_state_publisher,
        gazebo,
        spawn_entity
    ])
```

## File: ./src/rover_autonomy/rover_autonomy/__init__.py
```python

```

## File: ./src/rover_autonomy/rover_autonomy/brain_node.py
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class LunaboticsBrain(Node):
    def __init__(self):
        super().__init__('lunabotics_brain')
        
        # 1. Setup Navigation
        self.nav = BasicNavigator()
        
        # 2. Setup IR Sensor Subscriber
        self.bin_full = False
        self.create_subscription(Float32, '/ir_sensor_value', self.ir_callback, 10)
        
        # 3. Setup Digging Command Publisher
        self.dig_pub = self.create_publisher(Bool, '/cmd_dig', 10)

    def ir_callback(self, msg):
        # If reading is above threshold (10), mark as full
        if msg.data >= 10.0:
            self.bin_full = True

    def create_pose(self, x, y):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        return pose

    def run_autonomy_loop(self):
        # Wait for Nav2 to be ready
        self.nav.waitUntilNav2Active()

        while rclpy.ok():
            # STATE 1: Go to Dig Site
            self.get_logger().info("Moving to Dig Site...")
            dig_goal = self.create_pose(2.0, 1.0) # Change to your actual coords
            self.nav.goToPose(dig_goal)
            
            while not self.nav.isTaskComplete():
                pass # Driving...

            # STATE 2: Digging
            self.get_logger().info("Starting to Dig...")
            self.bin_full = False
            while not self.bin_full:
                # Publish 'True' to start the motors
                self.dig_pub.publish(Bool(data=True))
                # Spin once to check IR sensor data
                rclpy.spin_once(self, timeout_sec=0.1)
            
            # Stop Digging
            self.dig_pub.publish(Bool(data=False))
            self.get_logger().info("Bin Full! Moving to Offload...")

            # STATE 3: Go to Offload Site
            offload_goal = self.create_pose(0.0, 0.0)
            self.nav.goToPose(offload_goal)
            
            while not self.nav.isTaskComplete():
                pass # Driving to hopper...

            # STATE 4: Offloading (Timer based)
            self.get_logger().info("Offloading...")
            # Trigger your offload mechanism here
            import time
            time.sleep(5) # Simulate offload time
            
            self.get_logger().info("Cycle Complete. Restarting...")

def main():
    rclpy.init()
    brain = LunaboticsBrain()
    brain.run_autonomy_loop()

if __name__ == '__main__':
    main()
```

## File: ./src/rover_autonomy/urdf/assembled_robot.urdf
```xml
<?xml version="1.0"?>
<robot name="assembled_rover">

  <!-- ROOT LINK -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <box size="0.80 0.50 0.30"/>
      </geometry>
      <material name="dark_gray">
        <color rgba="0.25 0.25 0.25 1.0"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <box size="0.80 0.50 0.30"/>
      </geometry>
    </collision>

    <!-- Required inertial for root link in Gazebo -->
    <inertial>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <mass value="13.897"/>
      <inertia ixx="0.30" ixy="0.0" ixz="0.0"
               iyy="0.20" iyz="0.0" izz="0.25"/>
    </inertial>
  </link>

  <!-- Wheels – safe inertia (cylinder-like) -->
  <link name="WheelFR">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="2.152"/>
      <inertia ixx="0.012" ixy="0.0" ixz="0.0"
               iyy="0.018" iyz="0.0" izz="0.012"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.10" radius="0.15"/>
      </geometry>
      <material name="red">
        <color rgba="0.9 0.1 0.1 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.10" radius="0.15"/>
      </geometry>
    </collision>
  </link>

  <joint name="wheel_fr_joint" type="continuous">
    <parent link="base_link"/>
    <child link="WheelFR"/>
    <origin xyz="0.35 -0.25 -0.15" rpy="1.5708 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="WheelFL">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="2.152"/>
      <inertia ixx="0.012" ixy="0.0" ixz="0.0"
               iyy="0.018" iyz="0.0" izz="0.012"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.10" radius="0.15"/>
      </geometry>
      <material name="green">
        <color rgba="0.1 0.9 0.1 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.10" radius="0.15"/>
      </geometry>
    </collision>
  </link>

  <joint name="wheel_fl_joint" type="continuous">
    <parent link="base_link"/>
    <child link="WheelFL"/>
    <origin xyz="0.35 0.25 -0.15" rpy="1.5708 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="WheelRR">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="2.152"/>
      <inertia ixx="0.012" ixy="0.0" ixz="0.0"
               iyy="0.018" iyz="0.0" izz="0.012"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.10" radius="0.15"/>
      </geometry>
      <material name="blue">
        <color rgba="0.1 0.1 0.9 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.10" radius="0.15"/>
      </geometry>
    </collision>
  </link>

  <joint name="wheel_rr_joint" type="continuous">
    <parent link="base_link"/>
    <child link="WheelRR"/>
    <origin xyz="-0.25 -0.25 -0.15" rpy="1.5708 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="WheelRL">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="2.152"/>
      <inertia ixx="0.012" ixy="0.0" ixz="0.0"
               iyy="0.018" iyz="0.0" izz="0.012"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.10" radius="0.15"/>
      </geometry>
      <material name="yellow">
        <color rgba="0.9 0.9 0.1 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.10" radius="0.15"/>
      </geometry>
    </collision>
  </link>

  <joint name="wheel_rl_joint" type="continuous">
    <parent link="base_link"/>
    <child link="WheelRL"/>
    <origin xyz="-0.25 0.25 -0.15" rpy="1.5708 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <!-- Left Arm -->
  <link name="ArmL">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.315"/>
      <inertia ixx="0.006" ixy="0.0" ixz="0.0"
               iyy="0.006" iyz="0.0" izz="0.006"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.50 0.12 0.12"/>
      </geometry>
      <material name="cyan">
        <color rgba="0.0 0.8 0.9 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.50 0.12 0.12"/>
      </geometry>
    </collision>
  </link>

  <joint name="arm_l_joint" type="revolute">
    <parent link="base_link"/>
    <child link="ArmL"/>
    <origin xyz="0.30 0.20 0.35" rpy="1.5708 0 1.0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1.0"/>
  </joint>

  <!-- Left Drum -->
  <link name="DrumRevL">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="10.1"/>
      <inertia ixx="0.16" ixy="0.0" ixz="0.0"
               iyy="0.32" iyz="0.0" izz="0.16"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.60" radius="0.25"/>
      </geometry>
      <material name="orange">
        <color rgba="1.0 0.6 0.1 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.60" radius="0.25"/>
      </geometry>
    </collision>
  </link>

  <joint name="drum_l_joint" type="continuous">
    <parent link="ArmL"/>
    <child link="DrumRevL"/>
    <origin xyz="0 0.30 0" rpy="0 1.5708 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <!-- Right Arm -->
  <link name="ArmR">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.568"/>
      <inertia ixx="0.009" ixy="0.0" ixz="0.0"
               iyy="0.009" iyz="0.0" izz="0.009"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.55 0.14 0.14"/>
      </geometry>
      <material name="magenta">
        <color rgba="0.9 0.1 0.9 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.55 0.14 0.14"/>
      </geometry>
    </collision>
  </link>

  <joint name="arm_r_joint" type="continuous">
    <parent link="base_link"/>
    <child link="ArmR"/>
    <origin xyz="0.30 -0.20 0.35" rpy="1.5708 0 1.0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <!-- Right Drum -->
  <link name="DrumRevR">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="10.1"/>
      <inertia ixx="0.16" ixy="0.0" ixz="0.0"
               iyy="0.32" iyz="0.0" izz="0.16"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.60" radius="0.25"/>
      </geometry>
      <material name="lime">
        <color rgba="0.6 1.0 0.2 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.60" radius="0.25"/>
      </geometry>
    </collision>
  </link>

  <joint name="drum_r_joint" type="continuous">
    <parent link="ArmR"/>
    <child link="DrumRevR"/>
    <origin xyz="0 -0.30 0" rpy="0 1.5708 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <!-- ============================================= -->
  <!--          ros2_control configuration           -->
  <!-- ============================================= -->

  <!-- ROS2_CONTROL HARDWARE INTERFACE -->
  <ros2_control name="GazeboSystem" type="system">
    <hardware>
      <plugin>gz_ros2_control/GzSimSystem</plugin>   <!-- FIXED HERE -->
    </hardware>

    <!-- Left side wheels -->
    <joint name="wheel_fl_joint">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
    <joint name="wheel_rl_joint">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>

    <!-- Right side wheels -->
    <joint name="wheel_fr_joint">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
    <joint name="wheel_rr_joint">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
  </ros2_control>

  <!-- Gazebo plugin - keep this as-is since it loaded -->
  <gazebo>
    <plugin filename="libgz_ros2_control-system.so" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
      <parameters>$(find rover_autonomy)/config/ros2_controllers.yaml</parameters>
      <robot_param>robot_description</robot_param>
      <robot_param_node>robot_state_publisher</robot_param_node>
      <update_rate>100</update_rate>
    </plugin>
  </gazebo>
  
</robot>
```

