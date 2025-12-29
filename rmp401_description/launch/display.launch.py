from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_path

def generate_launch_description():
    use_gui   = DeclareLaunchArgument('use_gui',   default_value='true')
    with_zed4 = DeclareLaunchArgument('with_zed4', default_value='true')

    share = get_package_share_path('rmp401_description')
    xacro_file = str(share / 'urdf' / 'rmp401.urdf.xacro')

    # ⬇️ 공백(" ")을 꼭 넣어야 함!
    robot_desc_cmd = Command([
        FindExecutable(name='xacro'), " ",
        xacro_file, " ",
        "with_zed4:=", LaunchConfiguration('with_zed4')
    ])
    robot_description = ParameterValue(robot_desc_cmd, value_type=str)

    return LaunchDescription([
        use_gui, with_zed4,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(LaunchConfiguration('use_gui')),
            output='screen'
        ),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(share / 'rviz' / 'model_view.rviz')])
    ])
