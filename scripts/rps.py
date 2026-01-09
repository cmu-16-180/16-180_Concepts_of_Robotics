import mujoco
import mujoco.viewer
import numpy as np
import time
import os
import threading
import queue
import sys


# --- TERMINAL CLEANUP (Unix/Mac) ---
# If the daemon thread is killed while reading input, the terminal might
# be left in 'cbreak' mode (hidden cursor, no echo).
# We register an exit handler to restore sanity automatically.
if os.name != 'nt':
    import termios
    import tty
    import atexit
    try:
        _stdin_fd = sys.stdin.fileno()
        _orig_settings = termios.tcgetattr(_stdin_fd)
        
        def _restore_terminal():
            termios.tcsetattr(_stdin_fd, termios.TCSANOW, _orig_settings)
            
        atexit.register(_restore_terminal)
    except Exception:
        pass

# --- CROSS-PLATFORM INPUT HELPER ---
def get_key():
    """Reads a single keypress from stdin without waiting for Enter."""
    if os.name == 'nt':
        import msvcrt
        return msvcrt.getwch()
    else:
        # Unix implementation using global settings for safety
        fd = sys.stdin.fileno()
        try:
            tty.setcbreak(fd)
            ch = sys.stdin.read(1)
        finally:
            # Restore immediately for normal loop operation
            termios.tcsetattr(fd, termios.TCSADRAIN, _orig_settings)
        return ch
    

# --- INPUT WORKER ---
def input_loop(cmd_queue):
    """
    Runs in a background thread to read terminal input without blocking physics.
    """
    print("--------------------------------------------------")
    print(" INTERACTIVE MODE ACTIVE")
    print(" Type a key and hit ENTER to control the hand:")
    print("   'r' = ROCK")
    print("   'p' = PAPER")
    print("   's' = SCISSORS")
    print("   'a' = AUTO (Resume Sequence)")
    print("   'q' = QUIT")
    print("--------------------------------------------------")
    
    while True:
        try:
            # Read one character instantly
            cmd = get_key().lower()
            
            # Handle Ctrl+C (ETX) on Unix which sends \x03
            if cmd == '\x03': 
                cmd = 'q'
            
            if cmd:
                cmd_queue.put(cmd)
            
            if cmd == 'q':
                break
        except:
            break

def main():
    # --- CONFIGURATION ---
    robot_path = os.path.join("menagerie", "shadow_hand", "scene_left.xml")
    
    print(f"Loading robot: {robot_path}")
    if not os.path.exists(robot_path):
        print("[ERROR] Robot file not found! Did you run 'scripts/setup_menagerie.py'?")
        return

    try:
        model = mujoco.MjModel.from_xml_path(robot_path)
        data = mujoco.MjData(model)
    except Exception as e:
        print(f"[ERROR] {e}")
        return

    # --- SCENE TWEAKS ---
    
    # 1. HIDE THE EGG (Standard Menagerie Prop)
    # We look for a body named "object" or similar and banish it.
    for body_name in ["object", "target", "ellipsoid"]:
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if bid >= 0:
            # print(f"Hiding distraction object: '{body_name}'")
            model.body_pos[bid] = [10, 10, 0] # Move away

    # 2. ROTATE HAND TO FACE VIEWER
    hand_base_id = 1 # Usually the first body
    # print(f"Rotating Hand Base (Body {hand_base_id}) to face forward...")
    model.body_quat[hand_base_id] = [-0.707, 0.707, 0, 0]
    # Adjust position slightly up so it's centered
    model.body_pos[hand_base_id][2] += 0.2

    # --- PHYSICS SETUP ---
    model.opt.timestep = 0.002 

    # STABILITY: Inject Damping
    # print("Injecting Implicit Damping (0.1)...")
    for i in range(model.njnt):
        if model.jnt_type[i] == mujoco.mjtJoint.mjJNT_HINGE:
            model.dof_damping[model.jnt_dofadr[i]] = 0.5 

    # 2. Identify Joints
    fingers = ["FF", "MF", "RF", "LF"]
    joints = {}

    def get_joint_info(name):
        id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if id >= 0: 
            return (model.jnt_qposadr[id], model.jnt_dofadr[id])
        return None

    # Prefix Detection
    prefix = "lh_"
    if get_joint_info("rh_FFJ4") is not None: prefix = "rh_"
    # print(f"Detected Hand Prefix: '{prefix}'")

    for f in fingers:
        joints[f] = [
            get_joint_info(f"{prefix}{f}J3"),
            get_joint_info(f"{prefix}{f}J2"),
            get_joint_info(f"{prefix}{f}J1")
        ]
        joints[f + "_abd"] = [get_joint_info(f"{prefix}{f}J4")]

    joints["TH"] = [
        get_joint_info(f"{prefix}THJ5"),
        get_joint_info(f"{prefix}THJ4"),
        get_joint_info(f"{prefix}THJ3"),
        get_joint_info(f"{prefix}THJ2"),
        get_joint_info(f"{prefix}THJ1")
    ]

    # 3. Define Gestures
    def create_pose(): return {} 
    def set_pose(pose, finger_key, val):
        if finger_key in joints:
            for info in joints[finger_key]:
                if info:
                    _, dof_adr = info
                    pose[dof_adr] = val 

    # PAPER
    pose_paper = create_pose()
    for f in fingers: 
        set_pose(pose_paper, f, 0.0)
        set_pose(pose_paper, f+"_abd", 0.0)
    set_pose(pose_paper, "TH", 0.0)

    # ROCK
    pose_rock = create_pose()
    for f in fingers: 
        set_pose(pose_rock, f, 1.5)
        set_pose(pose_rock, f+"_abd", 0.0)
    set_pose(pose_rock, "TH", 1.5)

    # SCISSORS
    pose_scissors = create_pose()
    set_pose(pose_scissors, "RF", 1.5)
    set_pose(pose_scissors, "LF", 1.5)
    set_pose(pose_scissors, "TH", 1.5)
    set_pose(pose_scissors, "FF", 0.0)
    set_pose(pose_scissors, "MF", 0.0)
    set_pose(pose_scissors, "FF_abd", -0.1) 
    set_pose(pose_scissors, "MF_abd", 0.1)

    gestures = [pose_paper, pose_rock, pose_scissors]
    gesture_names = ["paper", "rock", "scissors"]

    kp = 2.0 
    
    print("Starting...")

    # --- THREADING SETUP ---
    cmd_queue = queue.Queue()
    input_thread = threading.Thread(target=input_loop, args=(cmd_queue,))
    input_thread.daemon = True # Kill thread when main program exits
    input_thread.start()

    with mujoco.viewer.launch_passive(model, data) as viewer:
        
        # --- APPLY DEFAULT CAMERA VIEW ---
        viewer.cam.azimuth = -132.91
        viewer.cam.elevation = -22.79
        viewer.cam.distance = 0.92
        viewer.cam.lookat = [-0.05, 0.15, 0.18]
        
        steps_per_frame = 8
        dt_render = 1.0 / 60.0
        last_cycle_idx = -1

        mode = "AUTO"
        manual_idx = 0
        last_display_state = ""
        
        while viewer.is_running():
            step_start = time.time()

            # --- CHECK INPUT ---
            try:
                while not cmd_queue.empty():
                    cmd = cmd_queue.get_nowait()
                    pose_key = {'r':1, 'p':0, 's':2}
                    if cmd in pose_key:
                        if mode != "MANUAL" or pose_key[cmd] != manual_idx:
                            mode = "MANUAL"
                            manual_idx = pose_key[cmd]
                            print(f"{gesture_names[manual_idx]} (manual)")
                    elif cmd == 'a':
                        if mode != "AUTO":
                            mode = "AUTO"
                            print("RESUMING AUTO MODE...")
                    elif cmd == 'q':
                        viewer.close()
            except:
                pass

            # --- "GAME" LOGIC ---
            # t = data.time
            # cycle_idx = int(t / 2.0) % 3
            
            # if cycle_idx != last_cycle_idx:
            #     print(f"{gesture_names[cycle_idx]}")
            #     last_cycle_idx = cycle_idx

            # target_pose = gestures[cycle_idx]
            if mode == "AUTO":
                t = data.time
                cycle_idx = int(t / 2.0) % 3
                target_pose = gestures[cycle_idx]
                
                # Print status only on change
                current_name = gesture_names[cycle_idx]
                if current_name != last_display_state:
                    print(f"{current_name} (auto)")
                    last_display_state = current_name
                    
            else:
                # Manual Mode
                target_pose = gestures[manual_idx]
                # Force last_display_state to update so if we switch back to auto 
                # it prints correctly
                last_display_state = "MANUAL"
            
            # --- PHYSICS LOOP ---
            for _ in range(steps_per_frame):
                data.ctrl[:] = 0
                for f_key in joints:
                    for info in joints[f_key]:
                        if info:
                            q_adr, dof_adr = info
                            target = target_pose.get(dof_adr, 0.0)
                            current = data.qpos[q_adr]
                            torque = kp * (target - current)
                            data.qfrc_applied[dof_adr] = torque
                mujoco.mj_step(model, data)

            viewer.sync()
            
            time_until_render = dt_render - (time.time() - step_start)
            if time_until_render > 0:
                time.sleep(time_until_render)
    
if __name__ == "__main__":
    main()