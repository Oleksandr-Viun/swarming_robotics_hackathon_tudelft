import os
import subprocess


subprocess.Popen(
        'ros2 action send_goal /mirte_master_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.6, max_effort: 30.0}}"',
        stdout=subprocess.DEVNULL,  # Ignore output
        stderr=subprocess.DEVNULL,  # Ignore errors
        stdin=subprocess.DEVNULL,   # Disconnect from parent input
        start_new_session=True,     # Make it independent of parent (POSIX),
        shell=True
    )
