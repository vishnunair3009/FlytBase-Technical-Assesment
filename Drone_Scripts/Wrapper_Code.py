# WRAPPER TEXT 

import argparse
import os
 
parser = argparse.ArgumentParser(
                    prog='python drone_collision_wrapper.py',
                    description='Generates a mission of drones and calls a program to check if any of them collide with the main drone')
 
parser.add_argument("--type", type=str,
                    help="Type of mission (random/highprob/guaranteed)")
 
 
args = parser.parse_args()
 
mission_generate_script = ""
mission_file = ""
 
if args.type == "random":
    mission_generate_script = "Completely_Random_Movement.py"
    mission_file = "missions.txt"
elif args.type == "highprob":
    mission_generate_script = "High_Probability_Collision_Generation.py"
    mission_file = "potential_collision_missions.txt"
elif args.type == "guaranteed":
    mission_generate_script = "Guarenteed_Collisions.py"
    mission_file = "guaranteed_collisions.txt"
 
if mission_generate_script == "":
    print("Invalid option for --type:")
    print("Enter one of the following: 'random'\t'highprob'\t'guaranteed'")
    exit()
else:
    os.system("python3 ./Drone_Scripts/%s" % (mission_generate_script))
 
os.system("python3 ./Drone_Scripts/Drone_Deconfliction_System_03.py %s" % (mission_file))