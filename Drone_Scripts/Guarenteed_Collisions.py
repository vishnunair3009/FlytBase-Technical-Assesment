# GUARENTEED COLLISIONS  

import random
import datetime
import os
import numpy as np # numpy needed for calculating distance and vector math
import math # For mathematical operations like sqrt

# --- Helper Functions (from previous code, slightly adjusted comments) ---

# Generating a random waypoint
def generate_random_waypoint(x_range=(0, 100), y_range=(0, 100), altitude_range=(0, 50)):
    """Generates a random 3D waypoint within specified per-axis ranges."""
    x = random.uniform(x_range[0], x_range[1])
    y = random.uniform(y_range[0], y_range[1])
    z = random.uniform(altitude_range[0], altitude_range[1])
    return (round(x, 2), round(y, 2), round(z, 2))

# Formatting mission data into a string from a list of waypoints
def format_mission_string(drone_name, base_time_offset_seconds, duration_seconds, waypoints):
    """Converts drone metadata and path into a formatted string."""
    if not waypoints:
        return "" # Return empty string for missions with no waypoints

    waypoints_str = ';'.join([f"{wp[0]},{wp[1]},{wp[2]}" for wp in waypoints])
    # Ensure all numbers are formatted as floats (e.g., 0.0 instead of 0)
    formatted_offset = f"{float(base_time_offset_seconds)}"
    formatted_duration = f"{float(duration_seconds)}"
    return f"{drone_name};{formatted_offset};{formatted_duration};{waypoints_str}"

# --- Mission File Generators ---

def generate_mission_file(filename="missions.txt", num_drones=10,
                          base_time_obj=datetime.datetime(2025, 5, 1, 12, 0, 0),
                          max_mission_duration_seconds=120,
                          coord_x_range=(0, 150), coord_y_range=(0, 150),
                          altitude_range=(0, 80),
                          min_waypoints_per_mission=2, max_waypoints_per_mission=5,
                          num_converging_drones=3):
    """
    Generates a mission file with:
    - A primary drone
    - Drones converging to a waypoint of the primary drone (potential, not guaranteed collision)
    - Randomly flying drones
    """
    # Adjust num_converging_drones if it's more than available drones
    if num_drones > 0:
        num_converging_drones = min(num_converging_drones, num_drones - 1)
    else:
        num_converging_drones = 0


    print(f"Generating {num_drones} missions (including {num_converging_drones} converging) to {filename}...")
    missions_data = []
    primary_mission_details = {}

    # 1. Primary drone
    if num_drones > 0:
        primary_name = "Alpha (Primary)"
        primary_start_offset = 0 # Primary starts immediately
        # Primary duration ensures it covers the timeframe for converging drones
        primary_duration = max(max_mission_duration_seconds // 2, 30) # Min 30s
        primary_duration = random.randint(primary_duration, max_mission_duration_seconds)

        primary_waypoints_count = random.randint(min_waypoints_per_mission, max_waypoints_per_mission)
        primary_waypoints_list = [generate_random_waypoint(coord_x_range, coord_y_range, altitude_range)
                                  for _ in range(primary_waypoints_count)]

        if primary_waypoints_list: # Only add if waypoints were generated
            primary_mission_str = format_mission_string(
                primary_name, primary_start_offset, primary_duration, primary_waypoints_list
            )
            missions_data.append(primary_mission_str)

            primary_mission_details.update({
                'waypoints': primary_waypoints_list,
                'start_offset': primary_start_offset,
                'duration': primary_duration,
                'num_waypoints': len(primary_waypoints_list)
            })
        else:
            print("Warning: Primary drone mission has no waypoints, skipping.")


    # Handle case where only 1 drone (the primary) was requested or primary failed
    if num_drones == 1 and primary_mission_details:
         num_converging_drones = 0 # Cannot have converging if only 1 drone
         num_random_to_generate = 0 # Cannot generate random if only 1 drone
    elif num_drones > 1 and not primary_mission_details:
         print("Warning: Primary drone failed to generate, skipping converging drones.")
         num_converging_drones = 0
         num_already_generated = 0 # No primary generated
         num_random_to_generate = num_drones # Generate all as random
    else:
         num_already_generated = (1 if primary_mission_details else 0) + num_converging_drones
         num_random_to_generate = max(0, num_drones - num_already_generated)


    # 2. Converging drones (designed for potential, not guaranteed, convergence)
    for i in range(num_converging_drones):
        drone_name = f"PotentialConvergeDrone_{i+1}"

        if not primary_mission_details.get('waypoints') or primary_mission_details.get('num_waypoints', 0) < 1:
             print(f"Warning: Primary drone has no waypoints for {drone_name} to converge towards. Generating random mission instead.")
             fallback_waypoints = [generate_random_waypoint(coord_x_range, coord_y_range, altitude_range)
                                  for _ in range(random.randint(min_waypoints_per_mission, max_waypoints_per_mission))]
             if fallback_waypoints:
                missions_data.append(format_mission_string(
                    drone_name,
                    random.randint(0, max_mission_duration_seconds // 2),
                    random.randint(max_mission_duration_seconds // 3, max_mission_duration_seconds),
                    fallback_waypoints
                ))
             continue


        # Choose a random target waypoint from the primary's path
        target_wp_index = random.randint(0, primary_mission_details['num_waypoints'] - 1)
        target_primary_wp = primary_mission_details['waypoints'][target_wp_index]

        # Estimate the time the primary drone reaches this waypoint
        # This is a simplification; the actual time depends on speed between waypoints
        # For simplicity, we assume equal time per segment between waypoints
        time_per_primary_segment = primary_mission_details['duration'] / max(1, primary_mission_details['num_waypoints'] - 1)
        approx_primary_time_at_target_wp_offset = primary_mission_details['start_offset'] + (target_wp_index * time_per_primary_segment)

        # Plan the converging drone's mission to pass through this waypoint roughly around that time
        conv_duration = random.randint(max(10, max_mission_duration_seconds // 4),
                                       max(30, int(max_mission_duration_seconds * 0.8)))
        # Start the converging drone's mission such that its midpoint (or a selected waypoint) aligns roughly with the primary's time
        # A simple approach: make the mission window overlap centered around the primary's target time
        approx_conv_start_offset = int(max(0, approx_primary_time_at_target_wp_offset - conv_duration / 2))
        # Add some randomness to the start time to make convergence less exact
        conv_start_offset = random.randint(max(0, approx_conv_start_offset - conv_duration // 4),
                                             approx_conv_start_offset + conv_duration // 4)
        conv_start_offset = max(0, conv_start_offset) # Ensure start offset is not negative

        conv_waypoints_count = random.randint(min_waypoints_per_mission, max_waypoints_per_mission)
        conv_waypoints_list = []

        # Define a radius around the target waypoint for generating convergence points
        convergence_radius_xy = (coord_x_range[1] - coord_x_range[0]) / 8
        convergence_radius_z = (altitude_range[1] - altitude_range[0]) / 8

        # Choose which waypoint in the converging drone's path will be near the target
        idx_for_target_in_conv_path = random.randint(0, max(0, conv_waypoints_count - 1))

        for k in range(conv_waypoints_count):
            if k == idx_for_target_in_conv_path:
                # Set this waypoint to be close to the primary's target waypoint
                # Use generate_random_waypoint but centered around the target waypoint
                 wp_x = target_primary_wp[0] + random.uniform(-convergence_radius_xy, convergence_radius_xy)
                 wp_y = target_primary_wp[1] + random.uniform(-convergence_radius_xy, convergence_radius_xy)
                 wp_z = target_primary_wp[2] + random.uniform(-convergence_radius_z, convergence_radius_z)

                 # Ensure the point is still within the overall bounds
                 wp_x = round(max(coord_x_range[0], min(coord_x_range[1], wp_x)), 2)
                 wp_y = round(max(coord_y_range[0], min(coord_y_range[1], wp_y)), 2)
                 wp_z = round(max(altitude_range[0], min(altitude_range[1], wp_z)), 2)

                 conv_waypoints_list.append((wp_x, wp_y, wp_z))
            else:
                # Other waypoints are just random within the full bounds
                 conv_waypoints_list.append(generate_random_waypoint(coord_x_range, coord_y_range, altitude_range))

        if conv_waypoints_list: # Only add if waypoints were generated
            missions_data.append(format_mission_string(
                drone_name, conv_start_offset, conv_duration, conv_waypoints_list
            ))
        else:
            print(f"Warning: Converging drone {drone_name} mission has no waypoints, skipping.")


    # 3. Random drones
    # num_already_generated count was adjusted above
    for i in range(num_random_to_generate):
        drone_name = f"RandomDrone_{i+1}"
        start_offset = random.randint(0, max_mission_duration_seconds // 2)
        duration = random.randint(max_mission_duration_seconds // 3, max_mission_duration_seconds)
        waypoints_count = random.randint(min_waypoints_per_mission, max_waypoints_per_mission)
        random_waypoints_list = [generate_random_waypoint(coord_x_range, coord_y_range, altitude_range)
                                 for _ in range(waypoints_count)]

        if random_waypoints_list: # Only add if waypoints were generated
            missions_data.append(format_mission_string(
                drone_name, start_offset, duration, random_waypoints_list
            ))
        else:
             print(f"Warning: Random drone {drone_name} mission has no waypoints, skipping.")


    # Write missions to file
    try:
        with open(filename, "w") as f:
            # Add a comment header
            f.write("# Drone Mission File Format: Name;StartOffsetSec;DurationSec;WP1_x,WP1_y,WP1_z;WP2_x,WP2_y,WP2_z;...\n")
            for mission_str in missions_data:
                if mission_str: # Ensure the line is not empty
                     f.write(mission_str + "\n")
        print(f"Successfully generated missions file: {filename} with {len(missions_data)} drone missions.")
    except IOError as e:
        print(f"Error writing mission file {filename}: {e}")


# --- Implementation for Guaranteed Collision ---

def generate_guaranteed_collision_mission(filename="guaranteed_collisions.txt", num_drones=3,
                                         base_time_obj=datetime.datetime(2025, 5, 3, 12, 0, 0),
                                         collision_point=(50.0, 50.0, 25.0),
                                         collision_time_offset_seconds=30.0,
                                         mission_duration_seconds=60.0,
                                         approach_distance=30.0, # Distance from collision point for start/end waypoints
                                         guaranteed_speed=5.0 # meters per second
                                         ):
    """
    Generates a mission file for multiple drones guaranteed to collide
    at a specific point and time.
    """
    if num_drones < 2:
        print("Need at least 2 drones for a guaranteed collision scenario.")
        # If file exists, clear it, otherwise ensure it doesn't get garbage data
        if os.path.exists(filename):
             open(filename, 'w').close()
        else:
             pass # File doesn't exist, nothing to clear
        return

    print(f"Generating {num_drones} guaranteed collision missions at {collision_point} at t+{collision_time_offset_seconds}s to {filename}...")
    missions_data = []
    collision_point_np = np.array(collision_point, dtype=float)

    if guaranteed_speed <= 0:
        print("Error: Guaranteed speed must be positive.")
        if os.path.exists(filename): open(filename, 'w').close()
        return

    # Calculate time needed to travel the approach distance at the guaranteed speed
    time_to_travel_approach = approach_distance / guaranteed_speed

    if time_to_travel_approach * 2 > mission_duration_seconds:
        print(f"Warning: Mission duration ({mission_duration_seconds}s) is too short to travel approach distance ({approach_distance}m) twice at {guaranteed_speed}m/s.")
        print(f"Minimum duration needed: {time_to_travel_approach * 2:.2f}s. Adjusting mission duration.")
        mission_duration_seconds = time_to_travel_approach * 2 + 5 # Add a buffer

    # Calculate the time offset for the start of the approach segment for each drone
    # Each drone must arrive at collision_point exactly at collision_time_offset_seconds
    # They start their approach segment time_to_travel_approach seconds before this
    approach_start_time_offset = collision_time_offset_seconds - time_to_travel_approach

    # Calculate the time offset for the end of their mission (after collision)
    # Each drone will leave the collision point at collision_time_offset_seconds
    # They will reach their end point time_to_travel_approach seconds after this
    mission_end_time_offset = collision_time_offset_seconds + time_to_travel_approach


    # Ensure the calculated mission times make sense
    if approach_start_time_offset < 0:
         print(f"Warning: Calculated approach start time offset ({approach_start_time_offset:.2f}s) is negative.")
         print("This means the mission would have to start before the base time to hit the collision point on time.")
         # Adjust collision_time_offset_seconds or approach_distance/speed
         # For now, just warn and proceed, the simulation will place the drone at the start waypoint until its start time.
         # A better approach might be to adjust approach_distance or collision_time_offset_seconds dynamically.
         pass # Let the start offset be negative, the simulator should handle it by holding at the start point.

    actual_duration_each_mission = mission_end_time_offset - approach_start_time_offset


    for i in range(num_drones):
        drone_name = f"CollisionDrone_{i+1}"

        # Generate a random approach direction vector
        # Use spherical coordinates or normalize a random 3D vector
        random_vector = np.random.uniform(-1.0, 1.0, 3)
        norm = np.linalg.norm(random_vector)
        if norm < 1e-6: # Avoid division by zero if random_vector is (0,0,0)
             random_vector = np.array([1.0, 0.0, 0.0]) # Default direction if random is zero
             norm = 1.0

        approach_direction_unit = random_vector / norm

        # Generate a slightly different random departure direction for the end point
        random_vector_end = np.random.uniform(-1.0, 1.0, 3)
        norm_end = np.linalg.norm(random_vector_end)
        if norm_end < 1e-6:
             random_vector_end = np.array([-1.0, 0.0, 0.0])
             norm_end = 1.0
        departure_direction_unit = random_vector_end / norm_end

        # Calculate start and end waypoints based on collision point, direction, and distance
        start_waypoint_np = collision_point_np - approach_direction_unit * approach_distance
        end_waypoint_np = collision_point_np + departure_direction_unit * approach_distance

        # Round waypoints for the mission file
        start_waypoint = tuple(round(c, 2) for c in start_waypoint_np)
        end_waypoint = tuple(round(c, 2) for c in end_waypoint_np)

        # Waypoints for this mission are: Start -> Collision Point -> End
        waypoints_list = [start_waypoint, tuple(round(c, 2) for c in collision_point), end_waypoint]

        missions_data.append(format_mission_string(
            drone_name,
            approach_start_time_offset, # Mission starts this many seconds after base_time_obj
            actual_duration_each_mission, # Total mission duration
            waypoints_list
        ))

    # Write missions to file
    try:
        with open(filename, "w") as f:
            # Add a comment header
            f.write("# Guaranteed Collision Mission File\n")
            f.write(f"# Collision Point: {collision_point}, Collision Time Offset: {collision_time_offset_seconds}s\n")
            f.write("# Format: Name;StartOffsetSec;DurationSec;WP1_x,WP1_y,WP1_z;WP2_x,WP2_y,WP2_z;...\n")
            for mission_str in missions_data:
                if mission_str:
                     f.write(mission_str + "\n")
        print(f"Successfully generated guaranteed collision missions file: {filename} with {len(missions_data)} drone missions.")
    except IOError as e:
        print(f"Error writing mission file {filename}: {e}")


# --- Main Execution ---
if __name__ == "__main__":
    # Example usage of the random/potential collision generator
    # This will create a file with a mix of random and potentially converging missions
    generate_mission_file(
        filename="potential_collision_missions.txt",
        num_drones=15, # Total drones
        num_converging_drones=5, # How many target the primary drone's path
        base_time_obj=datetime.datetime.now().replace(microsecond=0), # Use current time as base
        max_mission_duration_seconds=180,
        coord_x_range=(0, 300),
        coord_y_range=(0, 300),
        altitude_range=(0, 150),
        min_waypoints_per_mission=2,
        max_waypoints_per_mission=7
    )

    print("-" * 30) # Separator

    # Example usage of the GUARANTEED collision generator
    # This will create a file where drones ARE planned to collide
    guaranteed_collision_point = (100.0, 100.0, 50.0) # Where the collision should happen
    guaranteed_collision_time = 45.0 # Seconds after base_time_obj when collision occurs
    guaranteed_mission_length = 90.0 # Total duration of each drone's mission
    num_guaranteed_colliders = 5 # How many drones should collide

    generate_guaranteed_collision_mission(
        filename="guaranteed_collision_missions.txt",
        num_drones=num_guaranteed_colliders,
        base_time_obj=datetime.datetime.now().replace(microsecond=0), # Use current time as base
        collision_point=guaranteed_collision_point,
        collision_time_offset_seconds=guaranteed_collision_time,
        mission_duration_seconds=guaranteed_mission_length,
        approach_distance=50.0, # Drones start/end 50m away from collision point
        guaranteed_speed=10.0 # They travel at 10 m/s towards/away from the point
    )

    print("\nGenerated mission files.")
    