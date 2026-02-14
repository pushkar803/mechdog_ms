from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    world = PathJoinSubstitution([FindPackageShare("mechdog_gazebo"), "worlds", "mechdog_world.sdf"])
    xacro_file = PathJoinSubstitution([FindPackageShare("mechdog_description"), "urdf", "mechdog.urdf.xacro"])

    robot_description = {"robot_description": Command(["xacro ", xacro_file])}

    # Run gz sim HEADLESS to avoid OpenGL 3.3 issues in VMs
    # -r : run immediately
    # -s : server only (no GUI)
    gz = ExecuteProcess(
        cmd=["gz", "sim", "-r", "-s", world],
        output="screen"
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # Spawn into gz via ros_gz_sim (correct for new Gazebo)
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "mechdog",
            "-topic", "robot_description",
            "-x", "0", "-y", "0", "-z", "0.25"
        ],
        output="screen",
    )

    # Controller spawners (give gz + spawn time to initialize)
    jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    jgpc = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_group_position_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    delayed_spawn = TimerAction(period=3.0, actions=[spawn])
    delayed_ctrls = TimerAction(period=8.0, actions=[jsb, jgpc])

    return LaunchDescription([gz, rsp, delayed_spawn, delayed_ctrls, clock_bridge])
