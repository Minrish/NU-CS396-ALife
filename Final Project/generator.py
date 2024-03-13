import dm_control.mujoco
import mujoco.viewer
import numpy as np
import matplotlib.pyplot as plt
import time
import xml.etree.ElementTree as ET
import random
import os

def mutate_and_save(xml_file, offspring_file):
    tree = ET.parse(xml_file)
    root = tree.getroot()

    def constrain(value, min_val, max_val):
        return max(min_val, min(value, max_val))

    # Modify attributes for the bird's body with constraints
    bird_body_geom = root.find(".//body[@name='flappybird']/body/geom")
    if bird_body_geom is not None:
        sizes = bird_body_geom.get('size').split()
        new_sizes = [str(constrain(float(size) * random.uniform(0.75, 1.25), 0.1, 2.0)) for size in sizes]
        bird_body_geom.set('size', ' '.join(new_sizes))
        new_mass = constrain(float(bird_body_geom.get('mass')) * random.uniform(0.75, 1.25), 0.01, 2.0)
        bird_body_geom.set('mass', str(new_mass))

    # Modify attributes for the wings with constraints
    for geom in root.findall(".//body[@name='wing1']/geom") + root.findall(".//body[@name='wing2']/geom"):
        sizes = geom.get('size').split()
        new_sizes = [str(constrain(float(size) * random.uniform(0.75, 1.25), 0.1, 2.0)) for size in sizes]
        geom.set('size', ' '.join(new_sizes))
        new_mass = constrain(float(geom.get('mass')) * random.uniform(0.75, 1.25), 0.01, 2.0)
        geom.set('mass', str(new_mass))

    # Introduce variability in actuators' gear with constraints
    for motor in root.iter('motor'):
        new_gear = constrain(float(motor.get('gear')) * random.uniform(0.8, 1.2), 0.1, 100)
        motor.set('gear', str(new_gear))

    tree.write(offspring_file)

def evaluate_fitness(distance, stability_measure, energy_used, steps_num):
    # Checks for unrealistic values
    if np.isnan(distance) or np.isinf(distance) or distance <= 0 or np.isnan(stability_measure) or np.isinf(stability_measure) or np.isnan(energy_used) or np.isinf(energy_used):
        return -np.inf
    fitness = (distance * 2) - (np.sqrt(stability_measure) / steps_num) - (energy_used / 1000)
    return fitness

def run_simulation_with_viewer(xml_file):
    try:
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

                distance_moved = np.linalg.norm(data.qpos[:3] - initial_position[:3])
                total_distance += distance_moved
                initial_position = np.copy(data.qpos[:3])
                stability_measure += np.sum(data.qvel**2)

        fitness = evaluate_fitness(total_distance, stability_measure, energy_used, steps_num)
    except Exception as e:
        print(f"Simulation failed due to: {e}")
        fitness = -np.inf
    
    return fitness

def find_best_fitness_xml(fitness_scores, xml_files):
    best_fitness_index = np.argmax(fitness_scores)
    return xml_files[best_fitness_index], fitness_scores[best_fitness_index]

def run_evolutionary_process(initial_xml, generations=10, bodies_per_gen=10):
    best_fitness = -np.inf
    best_xml_file = initial_xml
    generation_fitness = []  # Track best fitness per generation for plotting

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
        generation_fitness.append(gen_best_fitness)  # Add best fitness of this generation to the list

        if gen_best_fitness > best_fitness:
            best_fitness = gen_best_fitness
            print(f"New best fitness due to mutation: {best_fitness}, adopting child {best_xml_file} as new base.")
        else:
            print(f"No improvement in generation {gen+1}, best fitness remains: {best_fitness}")

        for file in xml_files:
            if file != best_xml_file:
                os.remove(file)

    # Plotting the best fitness over generations
    plt.figure(figsize=(10, 6))
    plt.plot(range(1, generations + 1), generation_fitness, marker='o', linestyle='-', color='b')
    plt.title('Best Fitness per Generation')
    plt.xlabel('Generation')
    plt.ylabel('Best Fitness')
    plt.grid(True)
    plt.show()

# Example usage
if __name__ == "__main__":
    run_evolutionary_process("assets.xml", generations=5, bodies_per_gen=10)