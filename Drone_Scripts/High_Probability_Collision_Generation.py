# HIGH PROBABILITY COLLISION GENERATION 

import random
import datetime
import os
import numpy as np # numpy needed for calculating midpoint

# Generating a random waypoint 
def generate_random_waypoint(x_range=(0, 100), y_range=(0, 100), altitude_range=(0, 50)):
    """Generates a random 3D waypoint within specified per-axis ranges."""
# ranges specified for the x,y and z axis 
    x = random.uniform(x_range[0], x_range[1])
    y = random.uniform(y_range[0], y_range[1])
    z = random.uniform(altitude_range[0], altitude_range[1])

    return (round(x, 2), round(y, 2), round(z, 2))
# Picking a random float along each axis , roudning to 2 decimal places 
# and then returns a tuple.

# Formatting mission data into a string from a list of waypoints
def format_mission_string(drone_name, base_time_offset_seconds, duration_seconds, waypoints):
    waypoints_str = ';'.join([f"{wp[0]},{wp[1]},{wp[2]}" for wp in waypoints])
    # Format: name;start_time_offset_seconds;duration_seconds;x1,y1,z1;x2,y2,z2;...
    return f"{drone_name};{base_time_offset_seconds};{duration_seconds};{waypoints_str}"
# Conversion of drones metadata and path and formatting it to string 

def generate_mission_file(filename="missions.txt", num_drones=10, # Default 10 drones
                          base_time_obj=datetime.datetime(2025, 5, 1, 12, 0, 0),
                          max_mission_duration_seconds=120, # Increased default duration
                          coord_x_range=(0, 150), # Increased default range
                          coord_y_range=(0, 150), # Increased default range
                          altitude_range=(0, 80),  # Increased default altitude
                          min_waypoints_per_mission=2, max_waypoints_per_mission=5,
                          num_converging_drones=3): # Default to 3 converging drones
    """Generates a text file with multiple drone missions, including converging ones."""

# Initial sanity check - Ensure number of converging drones doesn't exceed total drones minus the primary
    if num_converging_drones >= num_drones and num_drones > 0:
        print(f"Warning: num_converging_drones ({num_converging_drones}) >= num_drones ({num_drones})."
              f" Setting converging to {max(0,num_drones - 1)}.")
        num_converging_drones = max(0, num_drones - 1)
    elif num_drones == 0:
         num_converging_drones = 0


    print(f"Generating {num_drones} missions (including {num_converging_drones} converging) to {filename}...")
    missions_data = []
    primary_mission_details = {}

# 1. Generate Primary Drone
    if num_drones > 0:
        primary_name = "Alpha (Primary)"
        primary_start_offset = 0
        primary_duration = random.randint(max_mission_duration_seconds // 2, max_mission_duration_seconds)
        primary_waypoints_count = random.randint(min_waypoints_per_mission, max_waypoints_per_mission)
        primary_waypoints_list = [generate_random_waypoint(coord_x_range, coord_y_range, altitude_range)
                                  for _ in range(primary_waypoints_count)]

        primary_mission_str = format_mission_string(
            primary_name, primary_start_offset, primary_duration, primary_waypoints_list
        )
        missions_data.append(primary_mission_str)

        primary_mission_details['waypoints'] = primary_waypoints_list
        primary_mission_details['start_offset'] = primary_start_offset
        primary_mission_details['duration'] = primary_duration
        primary_mission_details['num_waypoints'] = primary_waypoints_count
    else:
        # Handle case with 0 drones requested
        if os.path.exists(filename):
            open(filename, 'w').close() 
        print("No drones requested. Empty mission file (or cleared existing).")
        return

# 2. Generate Converging Drones
    for i in range(num_converging_drones):
        drone_name = f"ConvergeDrone_{i+1}"

        # To ensure that the primary drone has at least one waypoint to converge towards
        if not primary_mission_details.get('waypoints') or not primary_mission_details['waypoints']:
             print(f"Warning: Primary drone has no waypoints for {drone_name} to converge towards. Generating as random.")
             # Fallback to generating a random mission if primary has no waypoints
             start_offset_fallback = random.randint(0, max_mission_duration_seconds // 2)
             duration_fallback = random.randint(max_mission_duration_seconds // 3, max_mission_duration_seconds)
             wp_count_fallback = random.randint(min_waypoints_per_mission, max_waypoints_per_mission)
             fallback_waypoints = [generate_random_waypoint(coord_x_range, coord_y_range, altitude_range)
                                   for _ in range(wp_count_fallback)]
             missions_data.append(format_mission_string(
                 drone_name, start_offset_fallback, duration_fallback, fallback_waypoints
             ))
             continue 


        # Select a target waypoint on the primary drone's path
        # Choose a waypoint that is not the very last one if possible
        target_wp_index = random.randint(0, max(0, primary_mission_details['num_waypoints'] - 1))
        target_primary_wp = primary_mission_details['waypoints'][target_wp_index]

        # Calculating the approximate time that the primary drone reaches this waypoint
        time_per_primary_segment = primary_mission_details['duration'] / max(1, primary_mission_details['num_waypoints'] - 1)
        approx_primary_time_at_target_wp_offset = primary_mission_details['start_offset'] + (target_wp_index * time_per_primary_segment)

        # Determine duration and start time for the converging drone
        # Aim for the converging drone to be near the target waypoint around the same time
        conv_duration = random.randint(max(10, max_mission_duration_seconds // 4), max(30, int(max_mission_duration_seconds * 0.8)))
        # Start time should allow the converging drone to reach the target waypoint
        
        approx_conv_start_offset = int(max(0, approx_primary_time_at_target_wp_offset - conv_duration / 2))

        # Add some randomness to the start time offset
        conv_start_offset = random.randint(max(0, approx_conv_start_offset - conv_duration // 4), approx_conv_start_offset + conv_duration // 4)
        # Ensure start offset is not negative
        conv_start_offset = max(0, conv_start_offset)


        conv_waypoints_count = random.randint(min_waypoints_per_mission, max_waypoints_per_mission)
        conv_waypoints_list = []
        # Define a radius around the target waypoint for the converging drone's path
        convergence_radius_xy = (coord_x_range[1] - coord_x_range[0]) / 8
        convergence_radius_z = (altitude_range[1] - altitude_range[0]) / 8

        # Choose an index in the converging drone's path to place the target waypoint
        idx_for_target_in_conv_path = random.randint(0, max(0, conv_waypoints_count - 1))

        for k in range(conv_waypoints_count):
            if k == idx_for_target_in_conv_path:
                # Place the target primary waypoint directly in the converging drone's path to check if collisions occur or not 
                conv_waypoints_list.append(target_primary_wp)
            else:
                # Generate random waypoints around the target point for other waypoints, these are are the random movement of drones 
                # other than the 3 assured collisions
                wp_x = target_primary_wp[0] + random.uniform(-convergence_radius_xy, convergence_radius_xy)
                wp_y = target_primary_wp[1] + random.uniform(-convergence_radius_xy, convergence_radius_xy)
                wp_z = target_primary_wp[2] + random.uniform(-convergence_radius_z, convergence_radius_z)

                # Ensure waypoints are within the overall mission bounds
                wp_x = round(max(coord_x_range[0], min(coord_x_range[1], wp_x)), 2)
                wp_y = round(max(coord_y_range[0], min(coord_y_range[1], wp_y)), 2)
                wp_z = round(max(altitude_range[0], min(altitude_range[1], wp_z)), 2)
                conv_waypoints_list.append((wp_x, wp_y, wp_z))

        # To make sure theres atleast one waypoint if conv_waypoints_count was 0
        if not conv_waypoints_list:
             conv_waypoints_list.append(generate_random_waypoint(coord_x_range, coord_y_range, altitude_range))


        missions_data.append(format_mission_string(
            drone_name, conv_start_offset, conv_duration, conv_waypoints_list
        ))

# 3. Generating Remaining Random Drones
    num_already_generated = 1 + num_converging_drones if num_drones > 0 else 0
    num_random_to_generate = max(0, num_drones - num_already_generated)

    for i in range(num_random_to_generate):
        drone_name = f"RandomDrone_{i+1}"
        start_offset = random.randint(0, max_mission_duration_seconds // 2)
        duration = random.randint(max_mission_duration_seconds // 3, max_mission_duration_seconds)
        waypoints_count = random.randint(min_waypoints_per_mission, max_waypoints_per_mission)
        # Generate random waypoints directly for the random drones
        random_waypoints_list = [generate_random_waypoint(coord_x_range, coord_y_range, altitude_range)
                                 for _ in range(waypoints_count)]

        missions_data.append(format_mission_string(
            drone_name, start_offset, duration, random_waypoints_list
        ))

    try:
        with open(filename, "w") as f:
            for mission_str in missions_data:
                f.write(mission_str + "\n")
        print(f"Successfully generated missions file: {filename} with {len(missions_data)} drone missions.")
    except IOError as e:
        print(f"Error writing mission file {filename}: {e}")

if __name__ == "__main__":
    # Generating Random Missions with Converging Drones ---
    generate_mission_file(
        filename="random_collision_missions.txt",
        num_drones=10, # Total number of drones -----> Can be changed as per requirement of tester  
        num_converging_drones=3, # Number of drones designed to converge with the primary -----> can be changed as per requirement of tester 
        base_time_obj=datetime.datetime(2025, 5, 3, 12, 0, 0),
        max_mission_duration_seconds=120,
        coord_x_range=(0, 200),
        coord_y_range=(0, 200),
        altitude_range=(0, 100),
        min_waypoints_per_mission=3,
        max_waypoints_per_mission=6
    )



