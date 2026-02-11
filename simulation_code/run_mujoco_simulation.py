# run_mujoco_simulation.py
import mujoco
import mujoco.viewer

from so101_mujoco_utils2 import set_initial_pose
from so101_mujoco_pid_utils import move_to_pose_pid, hold_position_pid

# NEW: plotter
from so101_mujoco_utils2 import RealtimeJointPlotter  # (or wherever you placed it)

MODEL_PATH = "model/scene_urdf.xml"

m = mujoco.MjModel.from_xml_path(MODEL_PATH)
d = mujoco.MjData(m)

starting_position = {
    "shoulder_pan":  -4.4003158666,
    "shoulder_lift": -92.2462050161,
    "elbow_flex":     89.9543738355,
    "wrist_flex":     55.1185398916,
    "wrist_roll":      0.0,
    "gripper":         0.0,
}

desired_zero = {
    "shoulder_pan":  0.0,
    "shoulder_lift": 0.0,
    "elbow_flex":    0.0,
    "wrist_flex":    0.0,
    "wrist_roll":    0.0,
    "gripper":       0.0,
}

set_initial_pose(m, d, starting_position)

# NEW: start the realtime plotter once
plotter = RealtimeJointPlotter(max_points=4000)
plotter.start(host="127.0.0.1", port=8050, update_ms=100)  # open http://127.0.0.1:8050


with mujoco.viewer.launch_passive(m, d) as viewer:
    move_to_pose_pid(m, d, viewer, desired_zero, duration=2.0, realtime=True, plotter=plotter)
    hold_position_pid(m, d, viewer, desired_zero, duration=2.0, realtime=True, plotter=plotter)

    move_to_pose_pid(m, d, viewer, starting_position, duration=2.0, realtime=True, plotter=plotter)
    hold_position_pid(m, d, viewer, starting_position, duration=2.0, realtime=True, plotter=plotter)

    
