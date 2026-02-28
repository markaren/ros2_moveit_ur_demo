# Moveit demo

Due to issues with ros2_control on windows, 
this demo uses a custom fake_controller mimicking of 
`ros2_control_node + mock_components/GenericSystem + joint_trajectory_controller + joint_state_broadcaster` in a single node.

This node should be replaced with the standard ros2_control stack for real hardware.