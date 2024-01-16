import dm_control.mujoco
import mujoco.viewer
import time

def main():
    """
    Main function to launch the Mujoco viewer and simulate marine life.
    """
    try:
        m = dm_control.mujoco.MjModel.from_xml_path("quadruped.xml")
        d = dm_control.mujoco.MjData(m)

        motor_names = ["motor_f_l", "motor_f_r", "motor_r_l", "motor_r_r"]
        motor_ids = [dm_control.mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, m_name) for m_name in motor_names]

        with mujoco.viewer.launch_passive(m, d) as viewer:
            # Set camera parameters
            viewer.cam.azimuth = 180
            viewer.cam.elevation = -20
            viewer.cam.distance = 3.0
            viewer.cam.lookat[:] = [0.0, 0.0, 0.75]

            for _ in range(10000):
                for i in motor_ids:
                    d.ctrl[i] = 1.0

                dm_control.mujoco.mj_step(m, d)
                viewer.sync()
                time.sleep(1/100)

            viewer.close()

    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    main()
