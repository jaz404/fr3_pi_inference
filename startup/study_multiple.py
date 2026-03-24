import os
import sys
import time
from os import path

import libtmux

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python study_multiple.py <user_id> <round>")
        exit(1)

    ARGS = sys.argv[1:]
    user_id = ARGS[0]
    round = ARGS[1]
    PROTOCOL = "multiple"

    assert round in ["1", "2"], "Round must be either 1 or 2"

    if round == "1":
        checkpoint = "/home/user/intervention-learning/franka_ws/src/franka_flow/ckpts/USER_STUDY/base.ckpt"
    elif round == "2":
        checkpoint = f"/home/user/intervention-learning/franka_ws/src/franka_flow/ckpts/USER_STUDY/{user_id}/{PROTOCOL}/round1.ckpt"

    server = libtmux.Server(
        config_file=path.expandvars(
            "/home/user/intervention-learning/startup/.tmux.conf"
        )
    )
    if server.has_session("franka"):
        exit()
    else:
        session = server.new_session(
            "franka", start_directory="/home/user/intervention-learning", attach=False
        )

    # terminals for the simulation to start

    terminals = {
        "policy": f"ros2 launch franka_flow multiple_corrections.launch.py ckpt_path:={checkpoint}",
        "rqt": "rqt --perspective-file /home/user/intervention-learning/startup/default.perspective",
        "rviz": "rviz2 -d /home/user/intervention-learning/startup/pc.rviz",
        "franky_bringup": "ros2 launch franky_ros franky_bringup.launch.py",
        "realsense": "ros2 launch realsense_config pointcloud_rs.launch.py",
        # "xbox_control": "ros2 launch franky_ros franky_xbox.launch.py",
        "ps4_control": "ros2 launch franky_ros franky_ps4.launch.py",
        "recorder": f"ros2 launch il_recorder record.launch.py save_dir:=data/{user_id}/{PROTOCOL}/round{round} robot:=fr3_takeover.yaml",
    }

    for name, cmd in terminals.items():
        window = session.new_window(name, attach=False)
        window.select_layout(layout="tiled")
        pane = window.panes[0]
        time.sleep(0.1)
        pane.send_keys(cmd, suppress_history=True)
