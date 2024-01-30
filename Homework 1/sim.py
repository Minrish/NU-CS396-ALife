import time
import dm_control.mujoco
from dm_control import mjcf
import mujoco.viewer

def main():
    # Load the model from an XML file
    m = dm_control.mujoco.MjModel.from_xml_path("submarine.xml")
    d = dm_control.mujoco.MjData(m)

    # Start the viewer
    with mujoco.viewer.launch_passive(m, d) as viewer:

        # Set camera parameters for underwater view
        viewer.cam.azimuth = 180  # Azimuthal angle (in degrees)
        viewer.cam.elevation = -20  # Elevation angle (in degrees)
        viewer.cam.distance = 3.0  # Distance from the camera to the target
        viewer.cam.lookat[0] = 0.0  # X-coordinate of the target position
        viewer.cam.lookat[1] = 0.0  # Y-coordinate of the target position
        viewer.cam.lookat[2] = 0.75  # Z-coordinate of the target position

        for i in range(10000):
            # Simulate propeller thrust for forward motion
            propeller_name = "propeller_motor"  # Assuming a propeller motor is defined in your XML
            propeller_id = dm_control.mujoco.mj_name2id(m, mjcf.mjtObj.mjOBJ_ACTUATOR, propeller_name)
            d.ctrl[propeller_id] = 1.0  # Apply a constant force for the propeller

            # Simulate changes in buoyancy for depth control
            # Here, we'll simply alternate the submarine's depth for demonstration purposes
            depth_change_name = "ballast_tank_control"  # Assuming a ballast tank control is defined
            depth_change_id = dm_control.mujoco.mj_name2id(m, mjcf.mjtObj.mjOBJ_ACTUATOR, depth_change_name)
            if i % 200 < 100:
                d.ctrl[depth_change_id] = 1.0  # Increase buoyancy
            else:
                d.ctrl[depth_change_id] = -1.0  # Decrease buoyancy

            # Step the simulation
            dm_control.mujoco.mj_step(m, d)
            viewer.render()  # Render the scene in the viewer

            time.sleep(1/100)

        # Clean up
        viewer.close()

if __name__ == "__main__":
    main()
