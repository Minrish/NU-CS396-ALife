import dm_control.mujoco
import mujoco.viewer
import numpy as np
import time
import xml.etree.ElementTree as ET
import random

def modify_body(xml_file, new_file):
    tree = ET.parse(xml_file)
    root = tree.getroot()

    # Modify wing sizes and masses
    for wing in ['wing1', 'wing2']:
        geom = root.find(f".//body[@name='{wing}']/geom")
        new_length = random.uniform(0.2, 0.5)  # Adjusting the range for more variation
        new_mass = random.uniform(0.5, 2.0)    # Changing the mass of the wings
        size = geom.get('size').split()
        size[2] = str(new_length)
        geom.set('size', ' '.join(size))
        geom.set('mass', str(new_mass))

    tree.write(new_file)

def evaluate_fitness(distance, stability_measure, steps_num):
    # Adjusting the fitness function
    # Rewarding distance more and reducing the penalty for stability
    return distance - np.sqrt(stability_measure) / (10 * steps_num)

def run_simulation_with_viewer(xml_file):
    model = dm_control.mujoco.MjModel.from_xml_path(xml_file)
    data = dm_control.mujoco.MjData(model)

    angle_wing_span = 50
    freq_wing_span = 5
    steps_num = 500
    steps_time = 0.01
    angle_rad = np.deg2rad(angle_wing_span)

    initial_position = np.copy(data.qpos)
    total_distance = 0
    stability_measure = 0

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.azimuth = 90
        viewer.cam.elevation = -10
        viewer.cam.distance = 5.0
        viewer.cam.lookat[:] = [0.0, 0.0, 0.75]

        for s in range(steps_num):
            angle = angle_rad * np.sin(2 * np.pi * freq_wing_span * s * steps_time)

            # Enhanced wing movement control
            for i in range(2):
                data.ctrl[i] = angle * (-50 if i % 2 == 0 else 50)

            dm_control.mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(steps_time)

            # fitness metrics
            total_distance += np.linalg.norm(data.qpos[:2] - initial_position[:2])
            initial_position = np.copy(data.qpos)
            stability_measure += np.sum(data.qvel**2)

    fitness = evaluate_fitness(total_distance, stability_measure, steps_num)
    return fitness

# Procedural generation and evaluation
num_bodies = 10
for i in range(num_bodies):
    modify_body("assets.xml", f"modified_assets_{i}.xml")
    fitness = run_simulation_with_viewer(f"modified_assets_{i}.xml")
    print(f"Body {i} Fitness: {fitness}")
