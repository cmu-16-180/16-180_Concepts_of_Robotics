import mujoco
import mujoco.viewer
import numpy as np
import time
import os

def main():
    # --- CONFIGURATION ---
    robot_path = os.path.join("menagerie", "unitree_go1", "scene.xml")
    
    print(f"Loading robot: {robot_path}")
    if not os.path.exists(robot_path):
        print("[ERROR] Robot file not found! Did you run 'scripts/setup_menagerie.py'?")
        return

    # 1. Load Model
    try:
        model = mujoco.MjModel.from_xml_path(robot_path)
        data = mujoco.MjData(model)
    except Exception as e:
        print(f"[ERROR] {e}")
        return

    # --- PHYSICS SETUP ---
    model.opt.timestep = 0.002  # 500Hz for stability

    # INJECT IMPLICIT DAMPING (The stability trick)
    # This prevents the "bouncing" by handling damping in the solver.
    for i in range(model.njnt):
        if model.jnt_type[i] == mujoco.mjtJoint.mjJNT_HINGE:
            model.dof_damping[model.jnt_dofadr[i]] = 5.0 

    # 2. Set Initial State
    if model.nkey > 0:
        mujoco.mj_resetDataKeyframe(model, data, 0)
    else:
        data.qpos[2] = 0.35 # Default height

    q_home = data.qpos.copy()

    # 3. Identify Joints for Trot Gait
    legs = ["FR", "FL", "RR", "RL"]
    thigh_idxs = {}
    calf_idxs = {}
    
    def find_adr(base):
        for name in [base, f"{base}_joint"]:
            id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if id >= 0: return model.jnt_dofadr[id]
        return -1

    for leg in legs:
        thigh_idxs[leg] = find_adr(f"{leg}_thigh")
        calf_idxs[leg] = find_adr(f"{leg}_calf")

    # 4. Control Gains
    kp = 50.0   
    kd = 0.0    

    print("\nStarting Controller: MARCHING IN PLACE...")
    print("Press ESC to quit.")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        steps_per_frame = 8
        dt_render = 1.0 / 60.0

        viewer.cam.azimuth = 96.34
        viewer.cam.elevation = -44.42
        viewer.cam.distance = 4.09
        viewer.cam.lookat = [0.25, 0.03, -0.09]
        
        while viewer.is_running():
            step_start = time.time()

            # --- PHYSICS LOOP ---
            for _ in range(steps_per_frame):

                t = data.time

                # A. GENERATE GAIT (Trot in Place)
                # Experiment with the frequency of the gate (in Hz)
                freq = 1.5 # Was 2.0 Hz
                phase = (t * 2 * np.pi * freq)
                
                lift_A = max(0, np.sin(phase)) 
                lift_B = max(0, np.sin(phase + np.pi))
                
                q_target = q_home.copy()
                
                # Experiment with these parameters to change the gait motion
                scale_thigh = 0.5 # 0.5  # Was 0.6
                scale_calf = -0.5 # -1.0  # Was -1.2
                
                # Apply Group A (FR + RL)
                for leg in ["FR", "RL"]:
                    if thigh_idxs[leg] >= 0: q_target[thigh_idxs[leg]] += lift_A * scale_thigh
                    if calf_idxs[leg] >= 0:  q_target[calf_idxs[leg]]  += lift_A * scale_calf
                
                # Apply Group B (FL + RR)
                for leg in ["FL", "RR"]:
                    if thigh_idxs[leg] >= 0: q_target[thigh_idxs[leg]] += lift_B * scale_thigh
                    if calf_idxs[leg] >= 0:  q_target[calf_idxs[leg]]  += lift_B * scale_calf

                # B. RUN CONTROLLER
                for i in range(model.nu):
                    joint_id = model.actuator_trnid[i, 0]
                    if joint_id >= 0:
                        q_adr = model.jnt_qposadr[joint_id]
                        v_adr = model.jnt_dofadr[joint_id]
                        
                        # P-Control
                        err_pos = q_target[q_adr] - data.qpos[q_adr]
                        pd_torque = kp * err_pos 

                        # Gravity Compensation
                        gravity_comp = data.qfrc_bias[v_adr]
                        
                        gear = model.actuator_gear[i, 0]
                        if gear == 0: gear = 1 
                        data.ctrl[i] = (pd_torque + gravity_comp) / gear

                mujoco.mj_step(model, data)

            viewer.sync()
            
            time_until_render = dt_render - (time.time() - step_start)
            if time_until_render > 0:
                time.sleep(time_until_render)

    # ON EXIT: Print the final camera position so we can use it
    # to set up an initial view.
    # print("Final Camera Coordinates:")
    # print(f"viewer.cam.azimuth = {viewer.cam.azimuth:.2f}")
    # print(f"viewer.cam.elevation = {viewer.cam.elevation:.2f}")
    # print(f"viewer.cam.distance = {viewer.cam.distance:.2f}")
    # print(f"viewer.cam.lookat = [{viewer.cam.lookat[0]:.2f}, {viewer.cam.lookat[1]:.2f}, {viewer.cam.lookat[2]:.2f}]")


if __name__ == "__main__":
    main()