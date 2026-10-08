import mujoco
import mujoco.viewer
import numpy as np
import time
import glfw

XML_PATH = "robot_descriptions/franka/panda.xml"

model = mujoco.MjModel.from_xml_path(XML_PATH)
data = mujoco.MjData(model)

body_id = model.body("hand").id

print("Number of joints:", model.njnt)

for i in range(model.njnt):
    print(i, model.joint(i).name)


def keyboard_callback(key):

    step = np.deg2rad(5)

    # Panda arm: joints 0-6
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

    elif key == glfw.KEY_U:
        data.qpos[6] += step
    elif key == glfw.KEY_J:
        data.qpos[6] -= step


with mujoco.viewer.launch_passive(
        model, data, key_callback=keyboard_callback) as viewer:
    # Set camera
    viewer.cam.lookat[:] = [0.0, 0.0, 0.5]
    viewer.cam.distance = 2.0
    viewer.cam.azimuth = 90
    viewer.cam.elevation = -20

    while viewer.is_running():
	    mujoco.mj_forward(model, data)

	    # End-effector (hand) position
	    ee_pos = data.xpos[body_id]

	    print(
	        f"\rEnd-Effector Position: "
	        f"X = {ee_pos[0]:.4f}, "
	        f"Y = {ee_pos[1]:.4f}, "
	        f"Z = {ee_pos[2]:.4f}",
	        end=""
	    )

	    viewer.sync()
	    time.sleep(0.01)

