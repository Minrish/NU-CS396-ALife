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
    python generator.py
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

### Homework 4: Evolutionary Mutations and Stability

- **Objective**: Implement mutations in the physical attributes of the simulated bodies to explore the impact on fitness, focusing on mutations to body and wing sizes and masses.
- **Tasks**:
  1. Introduce mutations to the body and wing sizes and masses.
  2. Evaluate the impact of these mutations on the fitness scores across generations.
  3. Address simulation stability issues arising from extreme mutations.

- **Simulation Execution and Results**:
  - Run the generator script for evolutionary mutations:
    ```
    python generator.py
    ```
  - Observed results across 5 generations showed progressive improvements in fitness scores, with the best fitness due to mutations being notably higher in each subsequent generation. However, extreme mutations led to simulation instability in some instances, evidenced by `Nan`, `Inf`, or huge value warnings from the physics engine. These issues underscore the delicate balance between exploring the parameter space for optimal configurations and maintaining realistic, stable simulations.

- **Commentary on Simulation Stability**:
  - The evolutionary process led to both impressive improvements in fitness scores and instances of simulation instability. Particularly in Generation 4 and Generation 5, we observed `Nan`, `Inf`, or huge value warnings, highlighting the challenges of managing mutation magnitudes. This serves as a critical reminder of the importance of constraining mutations within realistic bounds and implementing mechanisms to detect and mitigate simulation instability. Future work will focus on refining mutation strategies to enhance simulation robustness while continuing to explore the parameter space for performance optimization. These issues will be addressed and hopefully resovled in Homework 5 submission. 