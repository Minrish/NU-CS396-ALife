import time
import dm_control.mujoco 
from mujoco.viewer import launch_passive

# Load the model from an XML file
m = dm_control.mujoco.MjModel.from_xml_path("example.xml")
d = dm_control.mujoco.MjData(m)

# Launch a passive viewer
viewer = launch_passive(m, d)

# Set camera parameters
viewer.cam.azimuth = 180  # Azimuthal angle (in degrees)
viewer.cam.elevation = -20  # Elevation angle (in degrees)
viewer.cam.distance = 3.0  # Distance from the camera to the target
viewer.cam.lookat[0] = 0.0  # X-coordinate of the target position
viewer.cam.lookat[1] = 0.0  # Y-coordinate of the target position
viewer.cam.lookat[2] = 0.75  # Z-coordinate of the target position

# Loop to step through the simulation
for _ in range(1000):
    dm_control.mujoco.mj_step(m, d)  # Advance the simulation
    viewer.sync()  # Synchronize the viewer
    time.sleep(0.01)  # Sleep for 1/100th of a second

# Close the viewer
viewer.close()
