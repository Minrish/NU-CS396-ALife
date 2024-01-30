import dm_control.mujoco
import mujoco.viewer
import numpy as np
import time

model = dm_control.mujoco.MjModel.from_xml_path("assets.xml")
data = dm_control.mujoco.MjData(model)

angle_wing_span = 50
freq_wing_span = 5

steps_num = 600
steps_time = 0.01

angle_rad = np.deg2rad(angle_wing_span)

with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.cam.azimuth = 90
    viewer.cam.elevation = -10
    viewer.cam.distance = 5.0
    viewer.cam.lookat[:] = [0.0, 0.0, 0.75]

    for s in range(steps_num):
        angle = angle_rad * np.sin(2 * np.pi * freq_wing_span * s * steps_time)

        for i in range(2):
            data.ctrl[i] = angle * (-20 if i % 2 == 0 else 20)
            seg_num = 1

            for j in range(seg_num):
                data.ctrl[2 + i * seg_num + j] = data.ctrl[i] / (j + 2)

        dm_control.mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(steps_time)
