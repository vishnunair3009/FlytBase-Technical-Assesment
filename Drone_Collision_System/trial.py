import random
import datetime
import os
import numpy as np  # numpy needed for calculating midpoint

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
    waypoints_str = ';'.join([f"{wp[0]},{wp[1]},{wp[2]}" for wp in waypoints])
    return f"{drone_name};{base_time_offset_seconds};{duration_seconds};{waypoints_str}"

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
    - Drones converging to a waypoint of the primary drone
    - Randomly flying drones
    """
    if num_converging_drones >= num_drones and num_drones > 0:
        print(f"Warning: num_converging_drones ({num_converging_drones}) >= num_drones ({num_drones})."
              f" Setting converging to {max(0, num_drones - 1)}.")
        num_converging_drones = max(0, num_drones - 1)
    elif num_drones == 0:
        num_converging_drones = 0

    print(f"Generating {num_drones} missions (including {num_converging_drones} converging) to {filename}...")
    missions_data = []
    primary_mission_details = {}

    # 1. Primary drone
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

        primary_mission_details.update({
            'waypoints': primary_waypoints_list,
            'start_offset': primary_start_offset,
            'duration': primary_duration,
            'num_waypoints': primary_waypoints_count
        })
    else:
        if os.path.exists(filename):
            open(filename, 'w').close()
        print("No drones requested. Empty mission file (or cleared existing).")
        return

    # 2. Converging drones
    for i in range(num_converging_drones):
        drone_name = f"ConvergeDrone_{i+1}"

        if not primary_mission_details.get('waypoints'):
            print(f"Warning: Primary drone has no waypoints for {drone_name} to converge towards.")
            fallback_waypoints = [generate_random_waypoint(coord_x_range, coord_y_range, altitude_range)
                                  for _ in range(random.randint(min_waypoints_per_mission, max_waypoints_per_mission))]
            missions_data.append(format_mission_string(
                drone_name,
                random.randint(0, max_mission_duration_seconds // 2),
                random.randint(max_mission_duration_seconds // 3, max_mission_duration_seconds),
                fallback_waypoints
            ))
            continue

        target_wp_index = random.randint(0, max(0, primary_mission_details['num_waypoints'] - 1))
        target_primary_wp = primary_mission_details['waypoints'][target_wp_index]

        time_per_primary_segment = primary_mission_details['duration'] / max(1, primary_mission_details['num_waypoints'] - 1)
        approx_primary_time_at_target_wp_offset = primary_mission_details['start_offset'] + (target_wp_index * time_per_primary_segment)

        conv_duration = random.randint(max(10, max_mission_duration_seconds // 4),
                                       max(30, int(max_mission_duration_seconds * 0.8)))
        approx_conv_start_offset = int(max(0, approx_primary_time_at_target_wp_offset - conv_duration / 2))
        conv_start_offset = random.randint(max(0, approx_conv_start_offset - conv_duration // 4),
                                           approx_conv_start_offset + conv_duration // 4)
        conv_start_offset = max(0, conv_start_offset)

        conv_waypoints_count = random.randint(min_waypoints_per_mission, max_waypoints_per_mission)
        conv_waypoints_list = []

        convergence_radius_xy = (coord_x_range[1] - coord_x_range[0]) / 8
        convergence_radius_z = (altitude_range[1] - altitude_range[0]) / 8

        idx_for_target_in_conv_path = random.randint(0, max(0, conv_waypoints_count - 1))

        for k in range(conv_waypoints_count):
            if k == idx_for_target_in_conv_path:
                conv_waypoints_list.append(target_primary_wp)
            else:
                wp_x = target_primary_wp[0] + random.uniform(-convergence_radius_xy, convergence_radius_xy)
                wp_y = target_primary_wp[1] + random.uniform(-convergence_radius_xy, convergence_radius_xy)
                wp_z = target_primary_wp[2] + random.uniform(-convergence_radius_z, convergence_radius_z)

                wp_x = round(max(coord_x_range[0], min(coord_x_range[1], wp_x)), 2)
                wp_y = round(max(coord_y_range[0], min(coord_y_range[1], wp_y)), 2)
                wp_z = round(max(altitude_range[0], min(altitude_range[1], wp_z)), 2)

                conv_waypoints_list.append((wp_x, wp_y, wp_z))

        if not conv_waypoints_list:
            conv_waypoints_list.append(generate_random_waypoint(coord_x_range, coord_y_range, altitude_range))

        missions_data.append(format_mission_string(
            drone_name, conv_start_offset, conv_duration, conv_waypoints_list
        ))

    # 3. Random drones
    num_already_generated = 1 + num_converging_drones
    num_random_to_generate = max(0, num_drones - num_already_generated)

    for i in range(num_random_to_generate):
        drone_name = f"RandomDrone_{i+1}"
        start_offset = random.randint(0, max_mission_duration_seconds // 2)
        duration = random.randint(max_mission_duration_seconds // 3, max_mission_duration_seconds)
        waypoints_count = random.randint(min_waypoints_per_mission, max_waypoints_per_mission)
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

# Placeholder function for guaranteed collision — should be implemented
def generate_guaranteed_collision_mission(filename, collision_point, collision_time_offset_seconds, mission_duration_seconds):
    print("This function needs to be implemented.")

if __name__ == "__main__":
    # Example usage
    generate_mission_file(
        filename="random_collision_missions.txt",
        num_drones=10,
        num_converging_drones=3,
        base_time_obj=datetime.datetime(2025, 5, 3, 12, 0, 0),
        max_mission_duration_seconds=120,
        coord_x_range=(0, 200),
        coord_y_range=(0, 200),
        altitude_range=(0, 100),
        min_waypoints_per_mission=3,
        max_waypoints_per_mission=6
    )

    generate_guaranteed_collision_mission(
        filename="guaranteed_collision_missions.txt",
        collision_point=(50, 50, 25),
        collision_time_offset_seconds=30,
        mission_duration_seconds=60
    )
