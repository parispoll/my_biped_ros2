

# contol_node = launch_ros.actions.Node(
#         package='controller_manager',
#         executable='ros2_control_node',
#         parameters=[
#             {'robot_description': robot_description},
#             ros2controlPath
#         ],
#         output='both'
#     )   

# joint_state_broadcaster_spawner = launch_ros.actions.Node(
#         package='controller_manager',
#         executable='spawner',
#         arguments=['joint_state_broadcaster'],
# )

# robot_controller_spawner = launch_ros.actions.Node(
#         package='controller_manager',
#         executable='spawner',
#         arguments=[
#             'leg_position_controller',
#             '--param-file', ros2controlPath
#         ],
# )

# nodeList=[gazebo, gazebo_bridge, rsp, contol_node, joint_state_broadcaster_spawner, robot_controller_spawner]