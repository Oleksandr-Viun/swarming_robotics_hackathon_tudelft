## Dropping
ros2 topic pub --once /mirte_master_arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
joint_names:
  - shoulder_pan_joint
  - shoulder_lift_joint
  - elbow_joint
  - wrist_joint
points:
  - positions:
      - 0.0
      - -0.3
      - -0.8
      - -1.0
    time_from_start:
      sec: 1.2
      nanosec: 0
"

## To scorpio
ros2 topic pub --once /mirte_master_arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
joint_names:
  - shoulder_pan_joint
  - shoulder_lift_joint
  - elbow_joint
  - wrist_joint
points:
  - positions:
      - 0.0
      - 1.2
      - -1.0
      - -1.2
    time_from_start:
      sec: 1.2
      nanosec: 0
"

mirte@mirte-3-0:~/workdir/library$ ros2 action send_goal /mirte_master_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: -0.6, max_effort: 50.0}}"
Waiting for an action server to become available...
Sending goal:
command:
position: -0.6
max_effort: 50.0

Goal accepted with ID: 48e3e1227d24464caf1dd4df779d4da9

Result:
position: -0.5256931781768799
effort: 50.0
stalled: true
reached_goal: false

Goal finished with status: ABORTED
^[[A^[[Amirte@mirte-3-0:~/workdir/library$ ros2 action send_goal /mirte_master_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.6, max_effort: 30.0}}"
Waiting for an action server to become available...
Sending goal:
command:
position: 0.6
max_effort: 30.0

Goal accepted with ID: 87af37b520674e2eb0134275024e69c8

Result:
position: 0.48380523920059204
effort: 30.0
stalled: true
reached_goal: false

Goal finished with status: ABORTED
