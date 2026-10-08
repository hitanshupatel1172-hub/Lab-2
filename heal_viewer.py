import mujoco
import mujoco.viewer
import numpy as np
import time
import glfw

XML_PATH = "robot_descriptions/single_arm_heal_effort_actuation_rs_mj.xml"

# Load HEAL
model = mujoco.MjModel.from_xml_path(XML_PATH)
data = mujoco.MjData(model)


# ==========================================
# KEYBOARD CONTROL
# ==========================================

def keyboard_callback(key):

    step = np.deg2rad(5)

    if key == glfw.KEY_Q:
        data.qpos[0] += step
    elif key == glfw.KEY_A:
        data.qpos[0] -= step

    elif key == glfw.KEY_W:
        data.qpos[1] += step
    elif key == glfw.KEY_S:
        data.qpos[1] -= step

    elif key == glfw.KEY_E:
        data.qpos[2] += step
    elif key == glfw.KEY_D:
        data.qpos[2] -= step

    elif key == glfw.KEY_R:
        data.qpos[3] += step
    elif key == glfw.KEY_F:
        data.qpos[3] -= step

    elif key == glfw.KEY_T:
        data.qpos[4] += step
    elif key == glfw.KEY_G:
        data.qpos[4] -= step

    elif key == glfw.KEY_Y:
        data.qpos[5] += step
    elif key == glfw.KEY_H:
        data.qpos[5] -= step
	
# ==========================================
# INITIALIZE
# ==========================================

mujoco.mj_forward(model, data)

# End-effector site
site_id = model.site("right_center").id


# ==========================================
# MUJOCO VIEWER
# ==========================================

with mujoco.viewer.launch_passive(
        model, data, key_callback=keyboard_callback) as viewer:

    while viewer.is_running():

        # Update MuJoCo kinematics
        mujoco.mj_forward(model, data)

        # Jacobian
        jac_pos = np.zeros((3, model.nv))
        jac_rot = np.zeros((3, model.nv))

        mujoco.mj_jacSite(
        model,
        data,
        jac_pos,
        jac_rot,
        site_id
        )

        # Clear terminal and print updated matrix
        print("\033[H\033[J", end="")

        print("========== HEAL ROBOT ==========")

        print("\nJoint Angles:")

        for i in range(6):
          print(f"J{i+1}: {np.rad2deg(data.qpos[i]):8.2f} deg")

          print("\nEnd-Effector Position:")
          print(f"X: {data.site_xpos[site_id][0]:8.4f} m")
          print(f"Y: {data.site_xpos[site_id][1]:8.4f} m")
          print(f"Z: {data.site_xpos[site_id][2]:8.4f} m")

          print("\nPosition Jacobian Jv:")
          print(jac_pos[:, :6])

          print("\n\nRotational Jacobian Jw:")
          print(jac_rot[:, :6])

        # Forward Kinematics
        x = data.site_xpos[site_id]

        print(
            f"\rEE Position: "
            f"X = {x[0]: .4f} m, "
            f"Y = {x[1]: .4f} m, "
            f"Z = {x[2]: .4f} m",
            end=""
        )

        viewer.sync()
        time.sleep(0.1)
