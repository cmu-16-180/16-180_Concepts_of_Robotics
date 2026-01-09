import mujoco
import mujoco.viewer
import numpy as np
import time

def main():
    # 100 is a "Sweet Spot" for laptop CPUs. 400 usually requires a desktop.
    box_count = 100
    print(f"Loading the 'Chaos Arm' demo with {box_count} boxes...")
    
    # 1. Define the Robot and World (Static XML)
    xml_header = """
    <mujoco>
      <!-- 240Hz Physics (0.00416s per step) -->
      <option gravity="0 0 -9.81" timestep="0.00416667"/>
      
      <!-- Increase memory limits for the contact solver to handle hundreds of boxes -->
      <size nconmax="2000" njmax="5000"/>

      <visual>
        <global offwidth="1920" offheight="1080"/>
      </visual>

      <!-- AESTHETICS: Textures and Materials -->
      <asset>
        <!-- Skybox for reflections -->
        <texture type="skybox" builtin="gradient" rgb1=".3 .5 .7" rgb2="0 0 0" width="32" height="512"/>
        
        <!-- The classic blue grid texture -->
        <texture name="grid" type="2d" builtin="checker" width="512" height="512" rgb1=".1 .2 .3" rgb2=".2 .3 .4"/>
        <material name="grid" texture="grid" texrepeat="1 1" texuniform="true" reflectance=".2"/>
        
        <!-- Robot Materials: Shiny and Metallic -->
        <material name="mat_base" rgba=".2 .2 .2 1" reflectance=".3" shininess=".8"/>
        <material name="mat_link1" rgba=".8 .1 .1 1" reflectance=".4" shininess=".6"/>
        <material name="mat_link2" rgba=".9 .4 .1 1" reflectance=".4" shininess=".6"/>
        <material name="mat_joint" rgba=".1 .1 .1 1" reflectance=".2" shininess=".9"/>
      </asset>

      <worldbody>
        <!-- Lighting and Floor -->
        <light pos="0 0 3" dir="0 0 -1" directional="true"/>
        <geom name="floor" type="plane" size="2 2 0.1" material="grid"/>

        <!-- The Robot Arm -->
        <body name="base" pos="0 0 0">
            <!-- Base Stand -->
            <geom type="cylinder" size=".1 .05" material="mat_base"/>
            
            <!-- Link 1 -->
            <body name="link1" pos="0 0 .1">
                <joint name="shoulder" type="hinge" axis="0 0 1" damping="0.5"/>
                
                <!-- Visual Decoration: The Motor Hub -->
                <geom type="cylinder" size=".06 .04" material="mat_joint"/>
                <!-- The Physical Arm Link -->
                <geom type="capsule" fromto="0 0 0 0.5 0 0" size=".04" material="mat_link1"/>
                
                <!-- Link 2 -->
                <body name="link2" pos="0.5 0 0">
                    <joint name="elbow" type="hinge" axis="0 0 1" damping="0.5"/>
                    
                    <!-- Visual Decoration: The Motor Hub -->
                    <geom type="cylinder" size=".05 .03" material="mat_joint"/>
                    <!-- The Physical Arm Link -->
                    <geom type="capsule" fromto="0 0 0 0.4 0 0" size=".03" material="mat_link2"/>
                    <!-- The "Fist" -->
                    <geom type="sphere" pos="0.4 0 0" size=".05" material="mat_joint"/> 
                </body>
            </body>
        </body>
    """

    # 2. Generate Boxes Programmatically
    boxes_xml = ""
    for i in range(box_count):
        # Position them in the arc of the arm (x: 0.4-0.8, y: -0.3 to 0.3)
        x = np.random.uniform(0.4, 0.8)
        y = np.random.uniform(-0.3, 0.3)
        z = np.random.uniform(0.1, 1.5) # Drop them from high up
        
        # Randomize Color
        r, g, b = np.random.uniform(0, 1, 3)
        
        boxes_xml += f"""
        <body name="box{i}" pos="{x} {y} {z}">
            <joint type="free"/>
            <geom type="box" size=".045 .045 .045" rgba="{r} {g} {b} 1"/>
        </body>
        """

    # 3. Define Actuators and Close Tags
    xml_footer = """
      </worldbody>
      <actuator>
        <!-- Servo Control: Slightly stiffer (300) to push through boxes -->
        <position joint="shoulder" kp="300" dampratio="1"/>
        <position joint="elbow" kp="300" dampratio="1"/>
      </actuator>
    </mujoco>
    """

    # Combine all parts
    full_xml = xml_header + boxes_xml + xml_footer

    # Load Model
    model = mujoco.MjModel.from_xml_string(full_xml)
    data = mujoco.MjData(model)

    print("Running simulation... (Press ESC in window to quit)")
    
    with mujoco.viewer.launch_passive(model, data) as viewer:
        
        viewer.cam.azimuth = 90.00
        viewer.cam.elevation = -45.00
        viewer.cam.distance = 3.15
        viewer.cam.lookat = [0.69, -0.00, -0.04]

        # We want 60Hz video.
        target_fps = 60.0
        time_per_frame = 1.0 / target_fps
        
        # Ideal physics steps per frame (at 240Hz, this is 4)
        target_physics_steps = 4
        
        while viewer.is_running():
            frame_start = time.time()

            # --- PHYSICS TIME BUDGET LOOP ---
            # We try to run 4 physics steps.
            # BUT, if we run out of time (computer is too slow), we stop early.
            # This turns "Lag/Freezing" into "Slow Motion" (which looks cool).
            
            for _ in range(target_physics_steps):
                
                # Update Control
                t = data.time
                data.ctrl[0] = 1.2 * np.sin(0.5 * t) 
                data.ctrl[1] = 1.0 * np.cos(0.5 * t)

                # Step Physics
                mujoco.mj_step(model, data)

                # CRITICAL: Check Time Budget
                # If we have spent more than 85% of our frame time (approx 14ms) calculating physics,
                # stop immediately and render so the UI stays responsive.
                if (time.time() - frame_start) > (time_per_frame * 0.85):
                    break

            # --- RENDER ---
            viewer.sync()
            
            # --- TIMING ---
            # If we were super fast, sleep to cap at 60fps.
            time_spent = time.time() - frame_start
            if time_spent < time_per_frame:
                time.sleep(time_per_frame - time_spent)

if __name__ == "__main__":
    main()