# FlytBase-Technical-Assesment
A Python system for verifying drone waypoint missions in shared airspace by detecting spatial and temporal conflicts with other drones. Features conflict explanations, scenario visualizations, and a 4D (3D+time) simulation. Designed for scalability and modularity focused on implementation in UTM (Unmanned Aircraft System Traffic Management).

# Installation Guide - How to Run ?

   1) Install requirements/ make sure all the dependencies are installed eg : pip install numpy matplotlib
   2) generate missions/ run the preliminary script for generating missions labelled "-----" using : python generate_missions.py 
   3) Run the main deconfliction system script using : python drone_deconfliction_system.py
# Setup and Execution Workflow 

2 step workflow !

   1) Mission Generation: First, run the mission generation script to create a realistic, diverse set of drone missions-including the primary and both random and converging simulated drones.

   2) Deconfliction & Visualization: Second, run the main system to check for spatial and temporal conflicts and produce visual outputs.

This seperation ensures modularity , making it easier to test with different scenarios and scale up for larger datasets, making it ideal for UTM type implementations. 


## Files

- `Drone_Collision_System_03.py`: Main script for the simulation.
- `Mission_Generation.py` : Script for randomised mission generation,i.e, convergence is intentional, but actual collision is probabilistic or randomized.
- `Guarenteed_Collisions_Mission_Generation.py`: Script for Guarenteed collision of drones to check if system is faulty or not.
- `Reflection.pdf`: A brief reflection on the project.
- `Test_Scenarios`: Contains two test cases — one with conflict, one without.
- `Outputs`: Video output files for demonstration.


