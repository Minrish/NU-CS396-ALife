import dm_control.mujoco
import mujoco.viewer
import numpy as np
import time
import xml.etree.ElementTree as ET
import random
import os

def mutate_and_save(xml_file, offspring_file):
    tree = ET.parse(xml_file)
    root = tree.getroot()

    # Modify existing attributes for the bird's body
    bird_body_geom = root.find(".//body[@name='flappybird']/body/geom")
    if bird_body_geom is not None:
        sizes = bird_body_geom.get('size').split()
        new_sizes = [str(float(size) * random.uniform(0.75, 1.25)) for size in sizes]
        bird_body_geom.set('size', ' '.join(new_sizes))
        new_mass = float(bird_body_geom.get('mass')) * random.uniform(0.75, 1.25)
        bird_body_geom.set('mass', str(new_mass))

    # Modify existing attributes for the wings
    for geom in root.findall(".//body[@name='wing1']/geom") + root.findall(".//body[@name='wing2']/geom"):
        sizes = geom.get('size').split()
        new_sizes = [str(float(size) * random.uniform(0.75, 1.25)) for size in sizes]
        geom.set('size', ' '.join(new_sizes))
        new_mass = float(geom.get('mass')) * random.uniform(0.75, 1.25)
        geom.set('mass', str(new_mass))

    # Introduce variability in actuators' gear without adding new limbs
    for motor in root.iter('motor'):
        new_gear = float(motor.get('gear')) * random.uniform(0.8, 1.2)
        motor.set('gear', str(new_gear))

    # Below are too agresive for implementation, will consider incorporating in next assignment. 
    # # Addition of new wings
    # if random.random() < 0.5:  # 50% chance to add a new wing
    #     new_wing_num = len(wings) + 1
    #     side = random.choice(["0 -90 0", "0 90 0"])
    #     new_wing = ET.SubElement(root.find('.//worldbody'), 'body', name=f'wing{new_wing_num}', pos=".5 -.3 0", euler=side)
    #     ET.SubElement(new_wing, 'joint', name=f'joint{new_wing_num}', type="hinge", axis="0 1 0", pos="0 0 .25", range="-45 45")
    #     ET.SubElement(new_wing, 'geom', type=random.choice(["box", "capsule"]), size="0.05 0.05 0.25", rgba="1 1 1 1", mass=str(random.uniform(0.5, 1.0)))

    # # Optional: Random removal of wings
    # if len(wings) > 2 and random.random() < 0.3:  # 30% chance to remove a wing if more than 2 exist
    #     root.find('.//worldbody').remove(random.choice(wings))

    tree.write(offspring_file)

def evaluate_fitness(distance, stability_measure, energy_used, steps_num):
    fitness = (distance * 2) - (np.sqrt(stability_measure) / steps_num) - (energy_used / 1000)
    return fitness

def run_simulation_with_viewer(xml_file):
    model = dm_control.mujoco.MjModel.from_xml_path(xml_file)
    data = dm_control.mujoco.MjData(model)

    angle_wing_span = 60
    freq_wing_span = 5
    steps_num = 500
    steps_time = 0.01
    angle_rad = np.deg2rad(angle_wing_span)
    energy_used = 0

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

            for i, ctrl in enumerate(data.ctrl):
                new_angle = angle * (-50 if i % 2 == 0 else 50)
                energy_used += abs(new_angle - ctrl)
                data.ctrl[i] = new_angle

            dm_control.mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(steps_time)

            total_distance += np.linalg.norm(data.qpos[:2] - initial_position[:2])
            initial_position = np.copy(data.qpos)
            stability_measure += np.sum(data.qvel**2)

    fitness = evaluate_fitness(total_distance, stability_measure, energy_used, steps_num)
    return fitness

def find_best_fitness_xml(fitness_scores, xml_files):
    best_fitness_index = np.argmax(fitness_scores)
    return xml_files[best_fitness_index], fitness_scores[best_fitness_index]

def run_evolutionary_process(initial_xml, generations=10, bodies_per_gen=10):
    best_fitness = -np.inf
    best_xml_file = initial_xml

    for gen in range(generations):
        print(f"Generation {gen+1}:")

        fitness_scores = []
        xml_files = []
        for i in range(bodies_per_gen):
            offspring_file = f"modified_assets_gen{gen}_body{i}.xml"
            mutate_and_save(best_xml_file, offspring_file)
            fitness = run_simulation_with_viewer(offspring_file)
            print(f"Body {i} Fitness: {fitness}")

            fitness_scores.append(fitness)
            xml_files.append(offspring_file)

        best_xml_file, gen_best_fitness = find_best_fitness_xml(fitness_scores, xml_files)
        if gen_best_fitness > best_fitness:
            best_fitness = gen_best_fitness
            print(f"New best fitness due to mutation: {best_fitness}, adopting child {best_xml_file} as new base.")
        else:
            print(f"No improvement in generation {gen+1}, best fitness remains: {best_fitness}")

        # Cleanup
        for file in xml_files:
            if file != best_xml_file:
                os.remove(file)

# Example usage
run_evolutionary_process("assets.xml", generations=5, bodies_per_gen=10)