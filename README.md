# NU-CS396-ALife: Artificial Life Coursework

This repository contains the homework assignments for the NU-CS396-ALife course, focusing on the simulation and study of artificial life. Each homework is designed to progressively build upon the concepts of simulating life-like behaviors and characteristics in a digital environment.

## Homework Assignments

### Homework 1: Basic Simulation

- **Objective**: Introduce a basic body structure into the simulation.
- **Tasks**:
  1. Create a simulated body.
  2. Implement movement capabilities for the body.

### Homework 2: Genotype to Phenotype Mapping

- **Objective**: Implement the directed graph genotype-to-phenotype conversion for creating 3D creatures, as described in Karl Sims's paper "Evolving 3D Morphology and Behavior by Competition."
- **Tasks**:
  1. Develop the genotype-to-phenotype mapping for 3D creature creation.

### Homework 3: Fitness Function and Iterative Simulation

- **Objective**: Optimize the fitness function to prioritize distance traveled and reduce the penalty for instability.
- **Tasks**:
  1. Modify the fitness function to favor distance more significantly and lessen stability penalties.
  2. Utilize parameters with the highest fitness scores from previous iterations as a baseline for subsequent iterations.
  3. Execute batch simulations for randomly generated configurations and evaluate their fitness.

- **Simulation Execution and Results**:
  - Run the generator script for each batch of configurations:
    ```
    (ArtificialLife) C:\Users\ybxYB\OneDrive\Northwestern\WINTER 2024\COMP_SCI_396\src\NU-CS396-ALife\Homework 3> python generator.py
    ```
  - Sample output (showing fitness scores for different bodies):
    ```
    Body 0 Fitness: 0.523
    Body 1 Fitness: 0.269
    ...
    Body 8 Fitness: 0.657
    ...
    Body 9 Fitness: 0.140
    ...
    Body 6 Fitness: 0.878
    ...
    Body 9 Fitness: 0.947
    ```
