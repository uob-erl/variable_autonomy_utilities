from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # Here we create argument objects. Remember _arg are objects not only values variables!
    # DeclareLaunchArgument allows you to expose the argument outside of your launch file.
    # LaunchConfiguration is local to the launch file and scoped.
    # DeclareLaunchArgument is an action which also needs to be added to LaunchDescription return

    # Joy node arg parameters
    deadzone_arg = DeclareLaunchArgument('deadzone', default_value='0.15')

    # Teleop mobile robot node arg parameters
    # If you need to change button mapping do it via parameters!
    # Do not hard code in .cpp !
    scaling_linear_arg = DeclareLaunchArgument(
        'scaling_linear', default_value='0.1')
    scaling_angular_arg = DeclareLaunchArgument(
        'scaling_angular', default_value='0.9')

    # Launching the nodes

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'deadzone': LaunchConfiguration('deadzone')}]
    )

    teleop_mobile_robot_node = Node(
        package='teleop_mobile_robot',
        executable='teleop_mobile_robot_node',
        name='teleop_mobile_robot_node',
        parameters=[{
                'scaling_linear': LaunchConfiguration('scaling_linear'),
                'scaling_angular': LaunchConfiguration('scaling_angular')
        }]
    )

    return LaunchDescription([
        deadzone_arg,
        scaling_linear_arg,
        scaling_angular_arg,
        joy_node,
        teleop_mobile_robot_node])
