#!/usr/bin/env bash
# =====================================================================
#  ROBODOG PLA (jaune/noir) - Gazebo Sim (Harmonic) + ROS 2 Jazzy
#  Colle tout ce fichier dans un terminal (ou: bash robodog_all.sh)
#  Sequence: REPOS -> LEVER -> MARCHE AVANT -> MARCHE ARRIERE -> REPOS
# =====================================================================
set -e
WS="$HOME/robotdog_ws"
source /opt/ros/jazzy/setup.bash

echo "### 1/5 Dependances"
sudo apt-get update -qq
sudo apt-get install -y ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers ros-jazzy-robot-state-publisher python3-numpy python3-colcon-common-extensions

echo "### 2/5 Workspace + paquet de description (meshes .dae/.stl)"
mkdir -p "$WS/src"
if [ ! -f "$WS/src/robotdog_description/package.xml" ]; then
  FOUND=$(find "$HOME" -maxdepth 6 -type f -path '*robotdog_description/package.xml' -not -path '*/install/*' -not -path '*/build/*' 2>/dev/null | head -1)
  if [ -z "$FOUND" ]; then
    echo "ERREUR: paquet robotdog_description introuvable (celui qui contient meshes/robotdog/*.dae et *.stl)."
    echo "Copie-le dans $WS/src/ puis relance ce script."; exit 1
  fi
  ln -sfn "$(dirname "$FOUND")" "$WS/src/robotdog_description"
  echo "robotdog_description trouve: $(dirname "$FOUND")"
fi

echo "### 3/5 Ecriture du paquet robotdog_sim"
rm -rf "$WS/src/robotdog_sim"
mkdir -p "$(dirname "$WS/src/robotdog_sim/package.xml")"
cat > "$WS/src/robotdog_sim/package.xml" <<'__ROBODOG_EOF__'
<?xml version="1.0"?>
<package format="3">
  <name>robotdog_sim</name>
  <version>1.0.0</version>
  <description>Robodog PLA: Gazebo Sim + trot gait</description>
  <maintainer email="robodog@example.com">robodog</maintainer>
  <license>MIT</license>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>python3-numpy</exec_depend>
  <exec_depend>trajectory_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>ros_gz_sim</exec_depend>
  <exec_depend>ros_gz_bridge</exec_depend>
  <exec_depend>gz_ros2_control</exec_depend>
  <exec_depend>controller_manager</exec_depend>
  <exec_depend>joint_state_broadcaster</exec_depend>
  <exec_depend>joint_trajectory_controller</exec_depend>
  <exec_depend>robotdog_description</exec_depend>
  <export><build_type>ament_python</build_type></export>
</package>
__ROBODOG_EOF__
mkdir -p "$(dirname "$WS/src/robotdog_sim/setup.py")"
cat > "$WS/src/robotdog_sim/setup.py" <<'__ROBODOG_EOF__'
import os
from glob import glob
from setuptools import setup

package_name = 'robotdog_sim'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robodog',
    maintainer_email='robodog@example.com',
    description='Robodog PLA: Gazebo Sim + trot gait',
    license='MIT',
    entry_points={'console_scripts': ['gait_node = robotdog_sim.gait_node:main']},
)
__ROBODOG_EOF__
mkdir -p "$(dirname "$WS/src/robotdog_sim/setup.cfg")"
cat > "$WS/src/robotdog_sim/setup.cfg" <<'__ROBODOG_EOF__'
[develop]
script_dir=$base/lib/robotdog_sim
[install]
install_scripts=$base/lib/robotdog_sim
__ROBODOG_EOF__
mkdir -p "$(dirname "$WS/src/robotdog_sim/resource/robotdog_sim")"
cat > "$WS/src/robotdog_sim/resource/robotdog_sim" <<'__ROBODOG_EOF__'
__ROBODOG_EOF__
mkdir -p "$(dirname "$WS/src/robotdog_sim/config/controllers.yaml")"
cat > "$WS/src/robotdog_sim/config/controllers.yaml" <<'__ROBODOG_EOF__'
controller_manager:
  ros__parameters:
    update_rate: 500
    use_sim_time: true

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

joint_trajectory_controller:
  ros__parameters:
    use_sim_time: true
    joints:
      - Revolute_2
      - Revolute_14
      - Revolute_4
      - Revolute_8
      - Revolute_9
      - Revolute_10
      - Revolute_11
      - Revolute_12
      - Revolute_13
      - Revolute_16
      - Revolute_17
      - Revolute_18
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    allow_partial_joints_goal: false
    open_loop_control: false
    allow_nonzero_velocity_at_trajectory_end: true
    constraints:
      stopped_velocity_tolerance: 0.0
      goal_time: 0.0
__ROBODOG_EOF__
mkdir -p "$(dirname "$WS/src/robotdog_sim/launch/sim.launch.py")"
cat > "$WS/src/robotdog_sim/launch/sim.launch.py" <<'__ROBODOG_EOF__'
import os
from launch import LaunchDescription
from launch.actions import (ExecuteProcess, TimerAction, IncludeLaunchDescription, RegisterEventHandler,
                            SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from robotdog_sim.urdf_tools import build_all


def generate_launch_description():
    urdf_text, planner, z_spawn, info = build_all()
    print('[robotdog_sim] info:', info)
    out = os.path.expanduser('~/robotdog_ws/robotdog_pla_rendered.urdf')
    try:
        open(out, 'w').write(urdf_text)      # copie lisible pour check_urdf / debug
    except Exception:
        pass

    desc_share = os.path.dirname(get_package_share_directory('robotdog_description'))
    old = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_res = desc_share + (':' + old if old else '')

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r -v 2 empty.sdf'}.items())

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               output='screen',
               parameters=[{'robot_description': urdf_text, 'use_sim_time': True}])

    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-topic', 'robot_description', '-name', 'robotdog',
                            '-x', '0', '-y', '0', '-z', '%.4f' % z_spawn])

    clock = Node(package='ros_gz_bridge', executable='parameter_bridge',
                 arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'], output='screen')

    jsb = Node(package='controller_manager', executable='spawner',
               arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'])
    jtc = Node(package='controller_manager', executable='spawner',
               arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'])
    gait = Node(package='robotdog_sim', executable='gait_node', output='screen',
                parameters=[{'use_sim_time': True}])

    # Camera: suit le robot de pres (3e personne). Repli: vue fixe proche si le suivi echoue.
    cam = ExecuteProcess(cmd=['bash', '-c',
        'gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean '
        '--timeout 3000 --req \'data: "robotdog"\' ; '
        'gz service -s /gui/follow/offset --reqtype gz.msgs.Vector3d --reptype gz.msgs.Boolean '
        '--timeout 3000 --req \'x: -0.9 y: -0.9 z: 0.55\''], output='screen')

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_res),
        gz, rsp, clock, spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[jtc])),
        RegisterEventHandler(OnProcessExit(target_action=jtc, on_exit=[gait])),
        TimerAction(period=6.0, actions=[cam]),
    ])
__ROBODOG_EOF__
mkdir -p "$(dirname "$WS/src/robotdog_sim/urdf/robotdog_pla.urdf.in")"
cat > "$WS/src/robotdog_sim/urdf/robotdog_pla.urdf.in" <<'__ROBODOG_EOF__'
<?xml version="1.0"?>
<robot name="robotdog">

  <!-- ===== PLA materials (3D-printed look) ===== -->
  <material name="PLA_Yellow"><color rgba="1.0 0.78 0.0 1.0"/></material>
  <material name="PLA_Black"><color rgba="0.04 0.04 0.04 1.0"/></material>

  <!-- Root frame: the CAD base frame has +Z pointing DOWN (legs on +Z side), so it is flipped here -->
  <link name="base_footprint"/>
  <joint name="footprint_to_base" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 0" rpy="@FLIP_RPY@"/>
  </joint>

  <link name="base_link">
    <inertial>
      <origin xyz="0.120097 0.034326 0.017053" rpy="0 0 0"/>
      <mass value="0.491594"/>
      <inertia ixx="7.862456e-04" ixy="-2.469420e-05" ixz="1.640229e-06"
               iyy="6.075689e-03" iyz="1.139014e-06" izz="6.779942e-03"/>
    </inertial>
    <visual>
      <origin xyz="0.000000 0.000000 0.000000" rpy="3.141593 0.000000 -3.141593"/>
      <geometry>
        <mesh filename="package://robotdog_description/meshes/robotdog/base_link.dae" scale="1 1 1"/>
      </geometry>
      <material name="PLA_Black"/>
    </visual>
    <collision>
      <origin xyz="0.000000 0.000000 0.000000" rpy="3.141593 0.000000 -3.141593"/>
      <geometry>
        <mesh filename="package://robotdog_description/meshes/robotdog/base_link_collision.stl" scale="0.01 0.01 0.01"/>
      </geometry>
    </collision>
  </link>

  <link name="HipFrontLeft_BackRight_f">
    <inertial>
      <origin xyz="-0.122865 -0.005647 -0.021543" rpy="0 0 0"/>
      <mass value="0.146657"/>
      <inertia ixx="9.324517e-05" ixy="0.000000e+00" ixz="0.000000e+00"
               iyy="8.510570e-04" iyz="-3.752093e-07" izz="8.132615e-04"/>
    </inertial>
    <visual>
      <origin xyz="-0.122865 0.011275 -0.136843" rpy="-1.570796 1.570796 0.000000"/>
      <geometry>
        <mesh filename="package://robotdog_description/meshes/robotdog/HipFrontLeft_BackRight_f.dae" scale="1 1 1"/>
      </geometry>
      <material name="PLA_Yellow"/>
    </visual>
    <collision>
      <origin xyz="-0.122865 0.011275 -0.136843" rpy="-1.570796 1.570796 0.000000"/>
      <geometry>
        <mesh filename="package://robotdog_description/meshes/robotdog/HipFrontLeft_BackRight_f_collision.stl" scale="0.01 0.01 0.01"/>
      </geometry>
    </collision>
  </link>

  <link name="HipFrontLeft_BackRight">
    <inertial>
      <origin xyz="0.122865 0.005647 -0.026976" rpy="0 0 0"/>
      <mass value="0.146657"/>
      <inertia ixx="9.324517e-05" ixy="0.000000e+00" ixz="0.000000e+00"
               iyy="8.510570e-04" iyz="3.752093e-07" izz="8.132615e-04"/>
    </inertial>
    <visual>
      <origin xyz="0.122865 -0.011275 -0.142276" rpy="1.570796 1.570796 0.000000"/>
      <geometry>
        <mesh filename="package://robotdog_description/meshes/robotdog/HipFrontLeft_BackRight.dae" scale="1 1 1"/>
      </geometry>
      <material name="PLA_Yellow"/>
    </visual>
    <collision>
      <origin xyz="0.122865 -0.011275 -0.142276" rpy="1.570796 1.570796 0.000000"/>
      <geometry>
        <mesh filename="package://robotdog_description/meshes/robotdog/HipFrontLeft_BackRight_collision.stl" scale="0.01 0.01 0.01"/>
      </geometry>
    </collision>
  </link>

  <link name="robotdog_HipFrontRight_BackLeft">
    <inertial>
      <origin xyz="0.059351 -0.005647 -0.021543" rpy="0 0 0"/>
      <mass value="0.073328"/>
      <inertia ixx="4.662258e-05" ixy="9.409761e-06" ixz="2.073234e-06"
               iyy="1.297163e-04" iyz="-1.876047e-07" izz="1.108186e-04"/>
    </inertial>
    <visual>
      <origin xyz="0.122865 0.011275 -0.136843" rpy="-1.570796 1.570796 0.000000"/>
      <geometry>
        <mesh filename="package://robotdog_description/meshes/robotdog/robotdog_HipFrontRight_BackLeft.dae" scale="1 1 1"/>
      </geometry>
      <material name="PLA_Yellow"/>
    </visual>
    <collision>
      <origin xyz="0.122865 0.011275 -0.136843" rpy="-1.570796 1.570796 0.000000"/>
      <geometry>
        <mesh filename="package://robotdog_description/meshes/robotdog/robotdog_HipFrontRight_BackLeft_collision.stl" scale="0.01 0.01 0.01"/>
      </geometry>
    </collision>
  </link>

  <link name="LowerSpine">
    <inertial>
      <origin xyz="0.122780 0.041593 0.006027" rpy="0 0 0"/>
      <mass value="0.406963"/>
      <inertia ixx="5.774892e-04" ixy="-3.431552e-08" ixz="3.626398e-07"
               iyy="4.735857e-03" iyz="1.654177e-12" izz="5.279430e-03"/>
    </inertial>
    <visual>
      <origin xyz="0.000000 0.000000 0.000000" rpy="0.000000 0.000000 0.000000"/>
      <geometry>
        <mesh filename="package://robotdog_description/meshes/robotdog/LowerSpine.dae" scale="1 1 1"/>
      </geometry>
      <material name="PLA_Yellow"/>
    </visual>
    <collision>
      <origin xyz="0.000000 0.000000 0.000000" rpy="0.000000 0.000000 0.000000"/>
      <geometry>
        <mesh filename="package://robotdog_description/meshes/robotdog/LowerSpine_collision.stl" scale="0.01 0.01 0.01"/>
      </geometry>
    </collision>
  </link>

  <link name="robotdog_HipFrontRight_BackLeft_2">
    <inertial>
      <origin xyz="-0.059351 0.005647 -0.026976" rpy="0 0 0"/>
      <mass value="0.073328"/>
      <inertia ixx="4.662258e-05" ixy="9.409761e-06" ixz="-2.073234e-06"
               iyy="1.297163e-04" iyz="1.876047e-07" izz="1.108186e-04"/>
    </inertial>
    <visual>
      <origin xyz="-0.122865 -0.011275 -0.142276" rpy="1.570796 1.570796 0.000000"/>
      <geometry>
        <mesh filename="package://robotdog_description/meshes/robotdog/robotdog_HipFrontRight_BackLeft_2.dae" scale="1 1 1"/>
      </geometry>
      <material name="PLA_Yellow"/>
    </visual>
    <collision>
      <origin xyz="-0.122865 -0.011275 -0.142276" rpy="1.570796 1.570796 0.000000"/>
      <geometry>
        <mesh filename="package://robotdog_description/meshes/robotdog/robotdog_HipFrontRight_BackLeft_2_collision.stl" scale="0.01 0.01 0.01"/>
      </geometry>
    </collision>
  </link>

  <link name="robotdog_FemurL_f">
    <inertial>
      <origin xyz="0.030044 0.032854 -0.021531" rpy="0 0 0"/>
      <mass value="0.068629"/>
      <inertia ixx="8.345821e-05" ixy="-3.689314e-05" ixz="2.999156e-07"
               iyy="8.066887e-05" iyz="8.956179e-07" izz="8.515317e-05"/>
    </inertial>
    <visual>
      <origin xyz="0.099707 0.120543 0.028748" rpy="1.570796 0.000000 0.785398"/>
      <geometry>
        <mesh filename="package://robotdog_description/meshes/robotdog/robotdog_FemurL_f.dae" scale="1 1 1"/>
      </geometry>
      <material name="PLA_Yellow"/>
    </visual>
    <collision>
      <origin xyz="0.099707 0.120543 0.028748" rpy="1.570796 0.000000 0.785398"/>
      <geometry>
        <mesh filename="package://robotdog_description/meshes/robotdog/robotdog_FemurL_f_collision.stl" scale="0.01 0.01 0.01"/>
      </geometry>
    </collision>
  </link>

  <link name="robotdog_FemurR_f">
    <inertial>
      <origin xyz="0.037523 -0.023958 0.028748" rpy="0 0 0"/>
      <mass value="0.137259"/>
      <inertia ixx="4.766375e-04" ixy="6.529546e-05" ixz="0.000000e+00"
               iyy="5.455924e-04" iyz="0.000000e+00" izz="1.703063e-04"/>
    </inertial>
    <visual>
      <origin xyz="0.127508 -0.090629 0.028748" rpy="-1.570796 0.000000 -0.523599"/>
      <geometry>
        <mesh filename="package://robotdog_description/meshes/robotdog/robotdog_FemurR_f.dae" scale="1 1 1"/>
      </geometry>
      <material name="PLA_Yellow"/>
    </visual>
    <collision>
      <origin xyz="0.127508 -0.090629 0.028748" rpy="-1.570796 0.000000 -0.523599"/>
      <geometry>
        <mesh filename="package://robotdog_description/meshes/robotdog/robotdog_FemurR_f_collision.stl" scale="0.01 0.01 0.01"/>
      </geometry>
    </collision>
  </link>

  <link name="robotdog_FemurR_f_2">
    <inertial>
      <origin xyz="0.032793 0.030110 -0.024843" rpy="0 0 0"/>
      <mass value="0.137259"/>
      <inertia ixx="5.010491e-04" ixy="-7.314967e-05" ixz="0.000000e+00"
               iyy="5.211808e-04" iyz="0.000000e+00" izz="1.703063e-04"/>
    </inertial>
    <visual>
      <origin xyz="0.109833 0.111394 -0.024843" rpy="1.570796 0.000000 0.698132"/>
      <geometry>
        <mesh filename="package://robotdog_description/meshes/robotdog/robotdog_FemurR_f_2.dae" scale="1 1 1"/>
      </geometry>
      <material name="PLA_Yellow"/>
    </visual>
    <collision>
      <origin xyz="0.109833 0.111394 -0.024843" rpy="1.570796 0.000000 0.698132"/>
      <geometry>
        <mesh filename="package://robotdog_description/meshes/robotdog/robotdog_FemurR_f_2_collision.stl" scale="0.01 0.01 0.01"/>
      </geometry>
    </collision>
  </link>

  <link name="robotdog_FemurL_f_2">
    <inertial>
      <origin xyz="0.037523 -0.023958 0.021531" rpy="0 0 0"/>
      <mass value="0.068629"/>
      <inertia ixx="6.482478e-05" ixy="3.264773e-05" ixz="-5.214991e-07"
               iyy="9.930228e-05" iyz="7.874766e-07" izz="8.515317e-05"/>
    </inertial>
    <visual>
      <origin xyz="0.127508 -0.090629 -0.028748" rpy="-1.570796 0.000000 -0.523599"/>
      <geometry>
        <mesh filename="package://robotdog_description/meshes/robotdog/robotdog_FemurL_f_2.dae" scale="1 1 1"/>
      </geometry>
      <material name="PLA_Yellow"/>
    </visual>
    <collision>
      <origin xyz="0.127508 -0.090629 -0.028748" rpy="-1.570796 0.000000 -0.523599"/>
      <geometry>
        <mesh filename="package://robotdog_description/meshes/robotdog/robotdog_FemurL_f_2_collision.stl" scale="0.01 0.01 0.01"/>
      </geometry>
    </collision>
  </link>

  <link name="TibiaR_f">
    <inertial>
      <origin xyz="-0.053021 -0.011526 0.026819" rpy="0 0 0"/>
      <mass value="0.062110"/>
      <inertia ixx="2.620096e-05" ixy="-3.623359e-05" ixz="2.695463e-06"
               iyy="1.611310e-04" iyz="4.977256e-07" izz="1.640661e-04"/>
    </inertial>
    <visual>
      <origin xyz="-0.152303 -0.023427 0.053193" rpy="-1.570796 0.000000 -2.879793"/>
      <geometry>
        <mesh filename="package://robotdog_description/meshes/robotdog/TibiaR_f.dae" scale="1 1 1"/>
      </geometry>
      <material name="PLA_Yellow"/>
    </visual>
    <collision>
      <origin xyz="-0.152303 -0.023427 0.053193" rpy="-1.570796 0.000000 -2.879793"/>
      <geometry>
        <mesh filename="package://robotdog_description/meshes/robotdog/TibiaR_f_collision.stl" scale="0.01 0.01 0.01"/>
      </geometry>
    </collision>
  </link>

  <link name="robotdog_TibiaL_f">
    <inertial>
      <origin xyz="-0.043182 -0.032854 -0.053193" rpy="0 0 0"/>
      <mass value="0.124221"/>
      <inertia ixx="2.425201e-04" ixy="-1.499434e-04" ixz="2.435278e-11"
               iyy="3.049568e-04" iyz="3.099340e-11" izz="3.281323e-04"/>
    </inertial>
    <visual>
      <origin xyz="-0.128133 -0.085598 -0.053193" rpy="-1.570796 0.000000 -2.443461"/>
      <geometry>
        <mesh filename="package://robotdog_description/meshes/robotdog/robotdog_TibiaL_f.dae" scale="1 1 1"/>
      </geometry>
      <material name="PLA_Yellow"/>
    </visual>
    <collision>
      <origin xyz="-0.128133 -0.085598 -0.053193" rpy="-1.570796 0.000000 -2.443461"/>
      <geometry>
        <mesh filename="package://robotdog_description/meshes/robotdog/robotdog_TibiaL_f_collision.stl" scale="0.01 0.01 0.01"/>
      </geometry>
    </collision>
  </link>

  <link name="robotdog_TibiaL_f_2">
    <inertial>
      <origin xyz="-0.051815 0.016103 0.049255" rpy="0 0 0"/>
      <mass value="0.124221"/>
      <inertia ixx="1.534420e-04" ixy="9.479661e-05" ixz="-3.348449e-11"
               iyy="3.940347e-04" iyz="2.079513e-11" izz="3.281323e-04"/>
    </inertial>
    <visual>
      <origin xyz="-0.149682 0.036612 0.049255" rpy="1.570796 0.000000 2.792527"/>
      <geometry>
        <mesh filename="package://robotdog_description/meshes/robotdog/robotdog_TibiaL_f_2.dae" scale="1 1 1"/>
      </geometry>
      <material name="PLA_Yellow"/>
    </visual>
    <collision>
      <origin xyz="-0.149682 0.036612 0.049255" rpy="1.570796 0.000000 2.792527"/>
      <geometry>
        <mesh filename="package://robotdog_description/meshes/robotdog/robotdog_TibiaL_f_2_collision.stl" scale="0.01 0.01 0.01"/>
      </geometry>
    </collision>
  </link>

  <link name="TibiaR">
    <inertial>
      <origin xyz="-0.047979 -0.024729 0.026897" rpy="0 0 0"/>
      <mass value="0.062502"/>
      <inertia ixx="5.362790e-05" ixy="-6.552626e-05" ixz="2.285197e-06"
               iyy="1.348369e-04" iyz="1.086107e-06" izz="1.650895e-04"/>
    </inertial>
    <visual>
      <origin xyz="-1.371584 0.864976 1.293193" rpy="-1.570796 0.000000 -0.523599"/>
      <geometry>
        <mesh filename="package://robotdog_description/meshes/robotdog/TibiaR.dae" scale="1 1 1"/>
      </geometry>
      <material name="PLA_Yellow"/>
    </visual>
    <collision>
      <origin xyz="-1.371584 0.864976 1.293193" rpy="-1.570796 0.000000 -0.523599"/>
      <geometry>
        <mesh filename="package://robotdog_description/meshes/robotdog/TibiaR_collision.stl" scale="0.01 0.01 0.01"/>
      </geometry>
    </collision>
  </link>

  <joint name="Revolute_2" type="revolute">
    <parent link="base_link"/>
    <child link="HipFrontLeft_BackRight_f"/>
    <origin xyz="0.284682 0.019937 0.029453" rpy="-1.570796 0.000000 1.570796"/>
    <axis xyz="0.000000 0.000000 1.000000"/>
    <limit lower="-0.6" upper="0.6" effort="12.0" velocity="8.0"/>
    <dynamics damping="0.05" friction="0.02"/>
  </joint>

  <joint name="Revolute_8" type="revolute">
    <parent link="base_link"/>
    <child link="HipFrontLeft_BackRight"/>
    <origin xyz="-0.039600 0.049280 0.029568" rpy="1.570796 0.000000 1.570796"/>
    <axis xyz="0.000000 0.000000 1.000000"/>
    <limit lower="-0.6" upper="0.6" effort="12.0" velocity="8.0"/>
    <dynamics damping="0.05" friction="0.02"/>
  </joint>

  <joint name="Revolute_11" type="revolute">
    <parent link="base_link"/>
    <child link="robotdog_HipFrontRight_BackLeft"/>
    <origin xyz="0.284682 0.049603 0.029453" rpy="-1.570796 0.000000 1.570796"/>
    <axis xyz="0.000000 0.000000 1.000000"/>
    <limit lower="-0.6" upper="0.6" effort="12.0" velocity="8.0"/>
    <dynamics damping="0.05" friction="0.02"/>
  </joint>

  <joint name="Rigid_15" type="fixed">
    <parent link="base_link"/>
    <child link="LowerSpine"/>
    <origin xyz="0.243584 -0.006273 0.062907" rpy="3.141593 0.000000 -3.141593"/>
  </joint>

  <joint name="Revolute_16" type="revolute">
    <parent link="base_link"/>
    <child link="robotdog_HipFrontRight_BackLeft_2"/>
    <origin xyz="-0.039600 0.019614 0.029568" rpy="1.570796 0.000000 1.570796"/>
    <axis xyz="0.000000 0.000000 1.000000"/>
    <limit lower="-0.6" upper="0.6" effort="12.0" velocity="8.0"/>
    <dynamics damping="0.05" friction="0.02"/>
  </joint>

  <joint name="Revolute_14" type="revolute">
    <parent link="HipFrontLeft_BackRight_f"/>
    <child link="robotdog_FemurL_f"/>
    <origin xyz="-0.103465 -0.035482 -0.020984" rpy="3.141593 1.570796 0.000000"/>
    <axis xyz="0.000000 0.000000 1.000000"/>
    <limit lower="-2.8" upper="2.8" effort="12.0" velocity="8.0"/>
    <dynamics damping="0.05" friction="0.02"/>
  </joint>

  <joint name="Revolute_9" type="revolute">
    <parent link="HipFrontLeft_BackRight"/>
    <child link="robotdog_FemurR_f"/>
    <origin xyz="0.103465 0.035482 -0.026417" rpy="-3.141593 -1.570796 0.000000"/>
    <axis xyz="0.000000 0.000000 1.000000"/>
    <limit lower="-2.8" upper="2.8" effort="12.0" velocity="8.0"/>
    <dynamics damping="0.05" friction="0.02"/>
  </joint>

  <joint name="Revolute_12" type="revolute">
    <parent link="robotdog_HipFrontRight_BackLeft"/>
    <child link="robotdog_FemurR_f_2"/>
    <origin xyz="0.103465 -0.035482 -0.020984" rpy="3.141593 1.570796 0.000000"/>
    <axis xyz="0.000000 0.000000 1.000000"/>
    <limit lower="-2.8" upper="2.8" effort="12.0" velocity="8.0"/>
    <dynamics damping="0.05" friction="0.02"/>
  </joint>

  <joint name="Revolute_17" type="revolute">
    <parent link="robotdog_HipFrontRight_BackLeft_2"/>
    <child link="robotdog_FemurL_f_2"/>
    <origin xyz="-0.104124 0.035482 -0.026417" rpy="-3.141593 -1.570796 0.000000"/>
    <axis xyz="0.000000 0.000000 1.000000"/>
    <limit lower="-2.8" upper="2.8" effort="12.0" velocity="8.0"/>
    <dynamics damping="0.05" friction="0.02"/>
  </joint>

  <joint name="Revolute_4" type="revolute">
    <parent link="robotdog_FemurL_f"/>
    <child link="TibiaR_f"/>
    <origin xyz="0.071457 0.071484 -0.000290" rpy="-3.141593 0.000000 0.000000"/>
    <axis xyz="0.000000 0.000000 1.000000"/>
    <limit lower="-2.8" upper="2.8" effort="12.0" velocity="8.0"/>
    <dynamics damping="0.05" friction="0.02"/>
  </joint>

  <joint name="Revolute_10" type="revolute">
    <parent link="robotdog_FemurR_f"/>
    <child link="robotdog_TibiaL_f"/>
    <origin xyz="0.087523 -0.050554 -0.000290" rpy="0.000000 0.000000 0.000000"/>
    <axis xyz="0.000000 0.000000 1.000000"/>
    <limit lower="-2.8" upper="2.8" effort="12.0" velocity="8.0"/>
    <dynamics damping="0.05" friction="0.02"/>
  </joint>

  <joint name="Revolute_13" type="revolute">
    <parent link="robotdog_FemurR_f_2"/>
    <child link="robotdog_TibiaL_f_2"/>
    <origin xyz="0.077415 0.064984 0.003065" rpy="0.000000 0.000000 0.000000"/>
    <axis xyz="0.000000 0.000000 1.000000"/>
    <limit lower="-2.8" upper="2.8" effort="12.0" velocity="8.0"/>
    <dynamics damping="0.05" friction="0.02"/>
  </joint>

  <joint name="Revolute_18" type="revolute">
    <parent link="robotdog_FemurL_f_2"/>
    <child link="TibiaR"/>
    <origin xyz="0.087523 -0.050554 0.003290" rpy="0.000000 0.000000 0.000000"/>
    <axis xyz="0.000000 0.000000 1.000000"/>
    <limit lower="-2.8" upper="2.8" effort="12.0" velocity="8.0"/>
    <dynamics damping="0.05" friction="0.02"/>
  </joint>

  <!-- PLA surface on the floor: PLA on PLA/plastic ~0.5, a little grip on the foot tips -->
  <gazebo reference="TibiaR_f"><mu1>1.0</mu1><mu2>1.0</mu2><kp>100000.0</kp><kd>100.0</kd></gazebo>
  <gazebo reference="robotdog_TibiaL_f"><mu1>1.0</mu1><mu2>1.0</mu2><kp>100000.0</kp><kd>100.0</kd></gazebo>
  <gazebo reference="robotdog_TibiaL_f_2"><mu1>1.0</mu1><mu2>1.0</mu2><kp>100000.0</kp><kd>100.0</kd></gazebo>
  <gazebo reference="TibiaR"><mu1>1.0</mu1><mu2>1.0</mu2><kp>100000.0</kp><kd>100.0</kd></gazebo>

  <ros2_control name="GazeboSimSystem" type="system">
    <hardware>
      <plugin>gz_ros2_control/GazeboSimSystem</plugin>
    </hardware>
    <joint name="Revolute_2">
      <command_interface name="position"/>
      <state_interface name="position"><param name="initial_value">@INIT_Revolute_2@</param></state_interface>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
    <joint name="Revolute_8">
      <command_interface name="position"/>
      <state_interface name="position"><param name="initial_value">@INIT_Revolute_8@</param></state_interface>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
    <joint name="Revolute_11">
      <command_interface name="position"/>
      <state_interface name="position"><param name="initial_value">@INIT_Revolute_11@</param></state_interface>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
    <joint name="Revolute_16">
      <command_interface name="position"/>
      <state_interface name="position"><param name="initial_value">@INIT_Revolute_16@</param></state_interface>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
    <joint name="Revolute_14">
      <command_interface name="position"/>
      <state_interface name="position"><param name="initial_value">@INIT_Revolute_14@</param></state_interface>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
    <joint name="Revolute_9">
      <command_interface name="position"/>
      <state_interface name="position"><param name="initial_value">@INIT_Revolute_9@</param></state_interface>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
    <joint name="Revolute_12">
      <command_interface name="position"/>
      <state_interface name="position"><param name="initial_value">@INIT_Revolute_12@</param></state_interface>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
    <joint name="Revolute_17">
      <command_interface name="position"/>
      <state_interface name="position"><param name="initial_value">@INIT_Revolute_17@</param></state_interface>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
    <joint name="Revolute_4">
      <command_interface name="position"/>
      <state_interface name="position"><param name="initial_value">@INIT_Revolute_4@</param></state_interface>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
    <joint name="Revolute_10">
      <command_interface name="position"/>
      <state_interface name="position"><param name="initial_value">@INIT_Revolute_10@</param></state_interface>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
    <joint name="Revolute_13">
      <command_interface name="position"/>
      <state_interface name="position"><param name="initial_value">@INIT_Revolute_13@</param></state_interface>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
    <joint name="Revolute_18">
      <command_interface name="position"/>
      <state_interface name="position"><param name="initial_value">@INIT_Revolute_18@</param></state_interface>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
  </ros2_control>

  <gazebo>
    <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
      <parameters>@CTRL_YAML@</parameters>
    </plugin>
  </gazebo>
</robot>
__ROBODOG_EOF__
mkdir -p "$(dirname "$WS/src/robotdog_sim/robotdog_sim/__init__.py")"
cat > "$WS/src/robotdog_sim/robotdog_sim/__init__.py" <<'__ROBODOG_EOF__'
__ROBODOG_EOF__
mkdir -p "$(dirname "$WS/src/robotdog_sim/robotdog_sim/kin.py")"
cat > "$WS/src/robotdog_sim/robotdog_sim/kin.py" <<'__ROBODOG_EOF__'
"""Cinematique du robotdog lue directement dans l'URDF (aucune longueur devinee)."""
import os, re, struct
import numpy as np
from xml.etree import ElementTree as ET

LEGS = {  # nom: (hanche, femur, genou, lien tibia)
    'A': ('Revolute_2',  'Revolute_14', 'Revolute_4',  'TibiaR_f'),
    'B': ('Revolute_8',  'Revolute_9',  'Revolute_10', 'robotdog_TibiaL_f'),
    'C': ('Revolute_11', 'Revolute_12', 'Revolute_13', 'robotdog_TibiaL_f_2'),
    'D': ('Revolute_16', 'Revolute_17', 'Revolute_18', 'TibiaR'),
}
PAIRS = (('A', 'B'), ('C', 'D'))      # trot: paires diagonales
ALL_JOINTS = [j for l in LEGS.values() for j in l[:3]]


def _rpy(a):
    x, y, z = a
    cx, sx, cy, sy, cz, sz = np.cos(x), np.sin(x), np.cos(y), np.sin(y), np.cos(z), np.sin(z)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _T(xyz, rpy):
    M = np.eye(4); M[:3, :3] = _rpy(rpy); M[:3, 3] = xyz
    return M


def _vec(s):
    return [float(v) for v in s.split()]


def read_stl(path):
    data = open(path, 'rb').read()
    if len(data) >= 84:
        n = struct.unpack('<I', data[80:84])[0]
        if 84 + 50 * n == len(data):
            a = np.frombuffer(data[84:], dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]), count=n)
            return a['v'].reshape(-1, 3).astype(float)
    txt = data.decode('ascii', 'ignore')
    return np.array([[float(x) for x in m] for m in re.findall(
        r'vertex\s+(\S+)\s+(\S+)\s+(\S+)', txt)])


class Robot:
    def __init__(self, urdf_path, pkg_share=None):
        self.root = ET.parse(urdf_path).getroot()
        self.pkg_share = pkg_share
        self.joints = {}
        for j in self.root.findall('joint'):
            o = j.find('origin'); ax = j.find('axis')
            self.joints[j.get('name')] = dict(
                parent=j.find('parent').get('link'), child=j.find('child').get('link'),
                T=_T(_vec(o.get('xyz')), _vec(o.get('rpy'))),
                axis=np.array(_vec(ax.get('xyz'))) if ax is not None else None)
        self.by_child = {j['child']: n for n, j in self.joints.items()}
        self.links = {l.get('name'): l for l in self.root.findall('link')}
        # lien "base_link" (repere CAD) = racine du bras cinematique
        self.base = 'base_link'
        self._tip = {}

    # ---------- FK ----------
    @staticmethod
    def _rot(axis, q):
        a = axis / np.linalg.norm(axis)
        K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
        M = np.eye(4); M[:3, :3] = np.eye(3) + np.sin(q) * K + (1 - np.cos(q)) * K @ K
        return M

    def link_T(self, link, q):
        """Transformation base_link -> link."""
        chain = []
        while link != self.base:
            n = self.by_child[link]; chain.append(n); link = self.joints[n]['parent']
        M = np.eye(4)
        for n in reversed(chain):
            j = self.joints[n]; M = M @ j['T']
            if j['axis'] is not None:
                M = M @ self._rot(j['axis'], q.get(n, 0.0))
        return M

    # ---------- maillages de collision ----------
    def collision_vertices(self, link):
        """Sommets de collision d'un lien, exprimes dans le repere du lien (metres)."""
        c = self.links[link].find('collision')
        if c is None or self.pkg_share is None:
            return None
        m = c.find('geometry/mesh')
        o = c.find('origin')
        rel = m.get('filename').split('package://')[1].split('/', 1)[1]
        pkg = m.get('filename').split('package://')[1].split('/')[0]
        path = os.path.join(self.pkg_share[pkg], rel) if isinstance(self.pkg_share, dict) else os.path.join(self.pkg_share, rel)
        if not os.path.exists(path):
            return None
        v = read_stl(path) * np.array(_vec(m.get('scale', '1 1 1')))
        T = _T(_vec(o.get('xyz')), _vec(o.get('rpy'))) if o is not None else np.eye(4)
        return (T[:3, :3] @ v.T).T + T[:3, 3]

    def tip_local(self, leg):
        """Point du pied dans le repere du tibia: sommet le plus eloigne de l'axe du genou."""
        if leg in self._tip:
            return self._tip[leg]
        tl = LEGS[leg][3]
        v = self.collision_vertices(tl)
        if v is None:   # secours si les STL sont introuvables
            ci = self.links[tl].find('inertial/origin')
            c = np.array(_vec(ci.get('xyz'))); c[2] = 0
            tip = np.append(c / np.linalg.norm(c) * 0.13, 0.0)[:3]
            self._tip[leg] = (tip, False)
            return self._tip[leg]
        r = np.hypot(v[:, 0], v[:, 1])
        sel = v[r > r.max() - 0.006]
        tip = sel.mean(axis=0)
        self._tip[leg] = (tip, True)
        return self._tip[leg]

    def foot(self, leg, q):
        tip, _ = self.tip_local(leg)
        return (self.link_T(LEGS[leg][3], q) @ np.append(tip, 1.0))[:3]

    # ---------- orientation (quel sens est le bas ?) ----------
    def down_sign(self):
        """+1 si le bas du chien est +Z dans le repere CAD (pieds du cote +Z du corps)."""
        feet = np.mean([self.foot(l, {})[2] for l in LEGS])
        zs = []
        for ln in ('base_link', 'LowerSpine'):
            v = self.collision_vertices(ln)
            if v is not None:
                zs.append(np.mean(v[:, 2]))
        body = np.mean(zs) if zs else 0.03
        return 1.0 if feet > body else -1.0

    # ---------- IK (plan sagittal, hanche = 0) ----------
    def ik(self, leg, target_xz, q0=(0.0, 0.0), iters=40):
        h, f, k, _ = LEGS[leg]
        q = np.array(q0, float)
        for _ in range(iters):
            p = self.foot(leg, {f: q[0], k: q[1]})
            e = np.array([target_xz[0] - p[0], target_xz[1] - p[2]])
            if np.linalg.norm(e) < 1e-5:
                break
            J = np.zeros((2, 2))
            for i in range(2):
                dq = q.copy(); dq[i] += 1e-5
                pp = self.foot(leg, {f: dq[0], k: dq[1]})
                J[:, i] = (np.array([pp[0], pp[2]]) - np.array([p[0], p[2]])) / 1e-5
            step = np.linalg.solve(J.T @ J + 1e-4 * np.eye(2), J.T @ e)
            n = np.linalg.norm(step)
            if n > 0.15: step *= 0.15 / n
            q += step
        p = self.foot(leg, {f: q[0], k: q[1]})
        err = float(np.hypot(target_xz[0] - p[0], target_xz[1] - p[2]))
        return q, err

    def ik3(self, leg, target, q0, iters=30):
        """IK 3D (hanche + femur + genou) vers une cible (x, y, z) dans le repere base_link."""
        h, f, kn, _ = LEGS[leg]
        q = np.array(q0, float)
        tgt = np.array(target, float)
        def pos(qq):
            return self.foot(leg, {h: qq[0], f: qq[1], kn: qq[2]})
        for _ in range(iters):
            p = pos(q); e = tgt - p
            if np.linalg.norm(e) < 1e-5:
                break
            J = np.zeros((3, 3))
            for i in range(3):
                dq = q.copy(); dq[i] += 1e-5
                J[:, i] = (pos(dq) - p) / 1e-5
            step = np.linalg.solve(J.T @ J + 1e-4 * np.eye(3), J.T @ e)
            n = np.linalg.norm(step)
            if n > 0.15: step *= 0.15 / n
            q += step
        return q, float(np.linalg.norm(tgt - pos(q)))
__ROBODOG_EOF__
mkdir -p "$(dirname "$WS/src/robotdog_sim/robotdog_sim/gait.py")"
cat > "$WS/src/robotdog_sim/robotdog_sim/gait.py" <<'__ROBODOG_EOF__'
"""Planificateur: repos -> debout -> marche avant -> marche arriere -> repos."""
import numpy as np
from .kin import Robot, LEGS, PAIRS, ALL_JOINTS

# ---------------- parametres a modifier librement ----------------
STAND_DEPTH = 0.19    # m, distance base_link -> sol (hauteur debout)
REST_DEPTH  = 0.10    # m, position repliee (couche)
STRIDE      = 0.07    # m, longueur de pas
STEP_H      = 0.035   # m, hauteur de levee du pied
PERIOD      = 0.80    # s, duree d'un cycle de trot
DUTY        = 0.62    # part du cycle ou le pied est au sol (>0.5 = recouvrement)
TURN_STRIDE = 0.07    # m, amplitude des pas (avant/arriere) pendant la rotation sur place
TURN_LAT    = 0.10    # m, amplitude laterale (pivot des hanches) pendant la rotation
TURN_90_CYCLES = 4.0  # cycles de pas pour tourner de 90 deg (<<< SEUL REGLAGE A AJUSTER >>>)
                      #   robot tourne trop peu -> augmente / trop -> diminue
TURN_DIR    = +1      # +1 ou -1 : sens de rotation
WALK_TIME   = 8.0     # s, duree de chaque marche
# -------------------------------------------------------------------


TURN_SIGN = {'A': +1, 'D': +1, 'B': -1, 'C': -1}   # pas avant/arriere: cotes opposes
LAT_SIGN  = {'A': +1, 'C': +1, 'B': -1, 'D': -1}   # pas lateraux: avant / arriere opposes


def smooth(t):
    t = np.clip(t, 0.0, 1.0)
    return 0.5 - 0.5 * np.cos(np.pi * t)


class Planner:
    def __init__(self, robot: Robot):
        self.R = robot
        self.nom = {}            # position nominale (x) du pied par patte
        self.nomy = {}
        for l in LEGS:
            p0 = robot.foot(l, {})
            self.nom[l], self.nomy[l] = p0[0], p0[1]
        self.q_stand, self.q_rest = {}, {}
        for l, (h, f, k, _) in LEGS.items():
            qs, es = robot.ik(l, (self.nom[l], STAND_DEPTH))
            qr, er = robot.ik(l, (self.nom[l], REST_DEPTH), q0=qs)
            self.q_stand[l], self.q_rest[l] = qs, qr
            assert es < 2e-3 and er < 2e-3, 'IK impossible pour la patte %s (%.4f, %.4f)' % (l, es, er)
        self._cache = {}

    # --- conversions ---
    def pose_dict(self, qlegs):
        d = {j: 0.0 for j in ALL_JOINTS}
        for l, (h, f, k, _) in LEGS.items():
            q = qlegs[l]
            if len(q) == 3:
                d[h], d[f], d[k] = float(q[0]), float(q[1]), float(q[2])
            else:
                d[f], d[k] = float(q[0]), float(q[1])
        return d

    def rest_pose(self):  return self.pose_dict(self.q_rest)
    def stand_pose(self): return self.pose_dict(self.q_stand)

    def blend(self, qa, qb, s):
        return {l: (1 - s) * qa[l] + s * qb[l] for l in LEGS}

    # --- marche ---
    def foot_target(self, leg, phase, cmd, lift=1.0):
        """phase dans [0,1): appui puis oscillation. cmd=(pas_x, pas_y) signes."""
        sx, sy = cmd
        x0, y0 = self.nom[leg], self.nomy[leg]
        if phase < DUTY:                       # appui: le pied recule par rapport au corps
            s = phase / DUTY
            f = 0.5 - s
            z = STAND_DEPTH
        else:                                  # oscillation
            s = (phase - DUTY) / (1 - DUTY)
            f = -0.5 + (s - np.sin(2 * np.pi * s) / (2 * np.pi))
            z = STAND_DEPTH - lift * STEP_H * np.sin(np.pi * s)
        return x0 + sx * f, y0 + sy * f, z

    def walk_pose(self, t, cmds, seed, lift=1.0):
        qs = {}
        for l in LEGS:
            ph = (t / PERIOD + (0.5 if l in PAIRS[1] else 0.0)) % 1.0
            x, y, z = self.foot_target(l, ph, cmds[l], lift)
            q, err = self.R.ik3(l, (x, y, z), seed[l], iters=25)
            qs[l] = q
        return qs

    # --- timeline complete ---
    def build(self, dt=0.02):
        """Retourne la liste (duree_totale, [(t, dict_articulations), ...], [(t, nom_phase), ...])."""
        seq = []; marks = []; t = 0.0
        cur = {l: self.q_rest[l].copy() for l in LEGS}

        def add(q, name=None):
            nonlocal t
            if name: marks.append((t, name))
            seq.append((t, self.pose_dict(q))); t += dt

        def hold(q, dur, name=None):
            n = max(1, int(dur / dt))
            for i in range(n):
                add(q, name if i == 0 else None)

        def move(qa, qb, dur, name=None):
            n = max(1, int(dur / dt))
            for i in range(n):
                add(self.blend(qa, qb, smooth((i + 1) / n)), name if i == 0 else None)
            return qb

        def walk(cmd_of, dur, name):
            n = int(dur / dt)
            seed = {l: np.array([0.0, self.q_stand[l][0], self.q_stand[l][1]]) for l in LEGS}
            ramp = 1.0 * PERIOD
            for i in range(n):
                tt = i * dt
                amp = smooth(tt / ramp) * smooth((dur - tt) / ramp)   # montee/descente douce
                q = self.walk_pose(tt, {l: (cmd_of[l][0] * amp, cmd_of[l][1] * amp) for l in LEGS},
                                   seed, min(1.0, 4.0 * amp))
                seed = q
                add(q, name if i == 0 else None)

        fwd = {l: (STRIDE, 0.0) for l in LEGS}
        turn = {l: (TURN_DIR * TURN_SIGN[l] * TURN_STRIDE, TURN_DIR * LAT_SIGN[l] * TURN_LAT) for l in LEGS}
        t90 = TURN_90_CYCLES * PERIOD + PERIOD       # + une periode de montee/descente
        t180 = 2 * TURN_90_CYCLES * PERIOD + PERIOD

        hold(self.q_rest, 4.0, 'REPOS')
        move(self.q_rest, self.q_stand, 3.5, 'LEVER')
        hold(self.q_stand, 1.5, 'DEBOUT')
        walk(fwd, WALK_TIME, 'MARCHE AVANT')
        hold(self.q_stand, 1.0, 'DEBOUT')
        walk(turn, t90, 'ROTATION 90 DEG')
        hold(self.q_stand, 1.0, 'DEBOUT')
        walk(fwd, WALK_TIME, 'MARCHE (NOUVELLE DIRECTION)')
        hold(self.q_stand, 1.0, 'DEBOUT')
        walk(turn, t180, 'ROTATION 180 DEG')
        hold(self.q_stand, 1.0, 'DEBOUT')
        walk(fwd, WALK_TIME, 'MARCHE (DIRECTION OPPOSEE)')
        hold(self.q_stand, 1.5, 'DEBOUT')
        move(self.q_stand, self.q_rest, 3.5, 'RETOUR AU REPOS')
        hold(self.q_rest, 2.0, None)
        return t, seq, marks

    def lowest_depth(self, q):
        """Distance entre base_link et le point de collision le plus bas, pour une pose donnee."""
        sgn = self.R.down_sign()
        zmax = max(sgn * self.R.foot(l, q)[2] for l in LEGS)   # au minimum les pieds
        for ln in self.R.links:
            v = self.R.collision_vertices(ln)
            if v is None:
                continue
            T = self.R.link_T(ln, q)
            w = (T[:3, :3] @ v.T).T + T[:3, 3]
            zmax = max(zmax, (sgn * w[:, 2]).max())
        return zmax
__ROBODOG_EOF__
mkdir -p "$(dirname "$WS/src/robotdog_sim/robotdog_sim/urdf_tools.py")"
cat > "$WS/src/robotdog_sim/robotdog_sim/urdf_tools.py" <<'__ROBODOG_EOF__'
import os, tempfile
import numpy as np
from ament_index_python.packages import get_package_share_directory
from .kin import Robot, ALL_JOINTS
from .gait import Planner

def template_path():
    return os.path.join(get_package_share_directory('robotdog_sim'), 'urdf', 'robotdog_pla.urdf.in')

def _fill(text, flip, q, ctrl):
    text = text.replace('@FLIP_RPY@', flip).replace('@CTRL_YAML@', ctrl)
    for j in ALL_JOINTS:
        text = text.replace('@INIT_%s@' % j, '%.5f' % q.get(j, 0.0))
    return text

def build_all():
    """Retourne (urdf_texte, planner, z_spawn, infos)."""
    raw = open(template_path()).read()
    ctrl = os.path.join(get_package_share_directory('robotdog_sim'), 'config', 'controllers.yaml')
    share = {'robotdog_description': get_package_share_directory('robotdog_description')}
    tmp = os.path.join(tempfile.gettempdir(), 'robotdog_probe.urdf')
    open(tmp, 'w').write(_fill(raw, '3.141593 0 0', {}, ctrl))
    robot = Robot(tmp, share)
    sign = robot.down_sign()
    planner = Planner(robot)
    flip = '3.141593 0 0' if sign > 0 else '0 0 0'
    rest = planner.rest_pose()
    z_spawn = planner.lowest_depth(rest) + 0.006
    text = _fill(raw, flip, rest, ctrl)
    tips = {l: robot.tip_local(l)[1] for l in 'ABCD'}
    info = dict(down_sign=sign, z_spawn=z_spawn, tip_from_stl=tips)
    return text, planner, z_spawn, info
__ROBODOG_EOF__
mkdir -p "$(dirname "$WS/src/robotdog_sim/robotdog_sim/gait_node.py")"
cat > "$WS/src/robotdog_sim/robotdog_sim/gait_node.py" <<'__ROBODOG_EOF__'
#!/usr/bin/env python3
"""Noeud de marche: REPOS -> LEVER -> MARCHE AVANT -> MARCHE ARRIERE -> REPOS."""
import numpy as np
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from .kin import ALL_JOINTS
from .urdf_tools import build_all

RATE = 50.0          # Hz de publication
LOOKAHEAD = 0.06     # s


class GaitNode(Node):
    def __init__(self):
        super().__init__('robotdog_gait')
        self.declare_parameter('loop', False)
        self.declare_parameter('start_delay', 2.0)
        _, planner, _, info = build_all()
        self.get_logger().info('Cinematique lue dans l\'URDF: %s' % info)
        total, seq, marks = planner.build(dt=0.02)
        self.dt = 0.02
        self.total = total
        self.table = np.array([[p[j] for j in ALL_JOINTS] for _, p in seq])
        self.marks = list(marks)
        self.pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.got_state = False
        self.create_subscription(JointState, '/joint_states', self._js, 10)
        self.t0 = None
        self.next_mark = 0
        self.timer = self.create_timer(1.0 / RATE, self.tick)
        self.get_logger().info('Plan pret: %.1f s. En attente du controleur...' % total)

    def _js(self, msg):
        self.got_state = True

    def _send(self, q):
        m = JointTrajectory()
        m.joint_names = ALL_JOINTS
        p = JointTrajectoryPoint()
        p.positions = [float(v) for v in q]
        p.time_from_start = Duration(sec=0, nanosec=int(LOOKAHEAD * 1e9))
        m.points = [p]
        self.pub.publish(m)

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            if not self.got_state or self.pub.get_subscription_count() == 0 or now < 1.0:
                self._send(self.table[0]) if self.pub.get_subscription_count() else None
                return
            self.t0 = now + float(self.get_parameter('start_delay').value)
            self.get_logger().info('Demarrage de la sequence.')
        t = now - self.t0
        if t < 0:
            self._send(self.table[0]); return
        if t > self.total - self.dt:
            if self.get_parameter('loop').value:
                self.t0 = now; self.next_mark = 0; return
            self._send(self.table[-1]); return
        while self.next_mark < len(self.marks) and t >= self.marks[self.next_mark][0]:
            self.get_logger().info('>>> %s' % self.marks[self.next_mark][1])
            self.next_mark += 1
        f = min((t + LOOKAHEAD) / self.dt, len(self.table) - 1.001)
        i = int(f); a = f - i
        self._send((1 - a) * self.table[i] + a * self.table[i + 1])


def main():
    rclpy.init()
    node = GaitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
__ROBODOG_EOF__

echo "### 4/5 Compilation"
cd "$WS"
colcon build --symlink-install --packages-up-to robotdog_sim
source "$WS/install/setup.bash"

echo "### 5/5 Lancement (Gazebo + controleurs + marche)"
pkill -f "gz sim" 2>/dev/null || true
sleep 1
ros2 launch robotdog_sim sim.launch.py
