

import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime, timedelta
from mpl_toolkits.mplot3d import Axes3D  # For 3D plotting
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button
import time
import itertools  # For itertools.count()
import os # Import os for file handling

class DroneMission:
    def __init__(self, waypoints, time_window, name="Drone", safety_buffer=2.0):
        # Ensure waypoints are tuples of floats
        self.waypoints = [tuple(float(c) for c in wp) for wp in waypoints]
        self.start_time, self.end_time = time_window
        self.name = name
        self.safety_buffer = safety_buffer

        # Calculate spatial points along the path
        self.spatial_points = self._calculate_path_points()
        self.points = self.spatial_points # Backward compatibility for check_collisions

        # Prepare the timed path based on spatial points and time window
        self.timed_path = []
        self._prepare_timed_path()

    def _calculate_path_points(self):
        """Calculates intermediate points along the path segments."""
        points = []
        if len(self.waypoints) < 2:
            if self.waypoints: points.append(self.waypoints[0])
            return points

        for i in range(len(self.waypoints)-1):
            start = np.array(self.waypoints[i], dtype=float)
            end = np.array(self.waypoints[i+1], dtype=float)
            distance = np.linalg.norm(end - start)

            if distance < 1e-6: # Treat as zero distance, add only if not duplicate of last point
                if not points or not np.allclose(points[-1], self.waypoints[i]):
                    points.append(self.waypoints[i])
                continue

            # Determine number of points based on segment distance for smoother paths
            # Adjust this factor (e.g., 2.5) for desired path resolution
            num_segment_points = max(2, int(distance * 2.5))
            # Generate points for the current segment, including endpoints
            current_segment_points = [tuple(start + t * (end - start)) for t in np.linspace(0, 1, num_segment_points, endpoint=True)]

            # Add points, avoiding duplicates at segment joins
            if i > 0 and points and np.allclose(points[-1], current_segment_points[0]):
                 points.extend(current_segment_points[1:])
            else:
                 points.extend(current_segment_points)

        # Final pass to remove any lingering duplicates (e.g., from waypoints being very close)
        if points:
            seen = set()
            unique_ordered_points = []
            # Using a small tolerance for floating point comparison in set
            tolerance = 1e-6
            for p_tuple in points:
                # Create a tuple of rounded values for hashing in the set
                rounded_p = tuple(round(c / tolerance) * tolerance for c in p_tuple)
                if rounded_p not in seen:
                    unique_ordered_points.append(p_tuple)
                    seen.add(rounded_p)
            points = unique_ordered_points

        # Ensure the very last waypoint is included if it's distinct from the last generated point
        if self.waypoints and points and not np.allclose(points[-1], self.waypoints[-1]):
             points.append(self.waypoints[-1])
        elif self.waypoints and not points: # Handle case with only one waypoint
             points.append(self.waypoints[0])

        return points


    def _prepare_timed_path(self):
        """Distributes spatial points over the mission time window."""
        self.duration_seconds = (self.end_time - self.start_time).total_seconds()
        self.timed_path = []

        if not self.spatial_points: return
        if self.duration_seconds < 0: self.duration_seconds = 0 # Handle negative duration

        if len(self.spatial_points) == 1:
            self.timed_path.append((self.start_time, self.spatial_points[0]))
            if self.duration_seconds > 0: # If duration > 0, drone stays at the point until end_time
                self.timed_path.append((self.end_time, self.spatial_points[0]))
            return

        if self.duration_seconds == 0: # If duration is zero, all points are at the start time
            for point_coord in self.spatial_points:
                self.timed_path.append((self.start_time, point_coord))
            return

        # Distribute points evenly across the duration
        # Time step is based on the number of *intervals* between points
        num_intervals = len(self.spatial_points) - 1
        time_step_seconds = self.duration_seconds / num_intervals

        for i, point_coord in enumerate(self.spatial_points):
            current_time = self.start_time + timedelta(seconds=i * time_step_seconds)
            self.timed_path.append((current_time, point_coord))


    def get_position_at_time(self, query_time_dt):
        """Gets the interpolated position of the drone at a specific datetime."""
        if not self.timed_path: return None

        # Handle times outside the mission window
        if query_time_dt < self.start_time: return self.timed_path[0][1] # Drone is at start point before mission
        if query_time_dt > self.end_time: return self.timed_path[-1][1] # Drone is at end point after mission

        # Handle single waypoint mission with duration
        if len(self.timed_path) == 2 and self.timed_path[0][1] == self.timed_path[1][1]:
             return self.timed_path[0][1] # Drone stays at the single point

        # Find the segment the query time falls into
        for i in range(len(self.timed_path) - 1):
            pt1_time, pt1_pos_tuple = self.timed_path[i]
            pt2_time, pt2_pos_tuple = self.timed_path[i+1]

            # Check if query_time_dt is exactly at the start of a segment
            if query_time_dt == pt1_time:
                return pt1_pos_tuple
            # Check if query_time_dt is within the current segment (inclusive of end time for the last segment)
            if pt1_time < query_time_dt <= pt2_time:
                segment_duration_seconds = (pt2_time - pt1_time).total_seconds()
                pt1_pos = np.array(pt1_pos_tuple, dtype=float)
                pt2_pos = np.array(pt2_pos_tuple, dtype=float)

                if segment_duration_seconds == 0: # Points are at the same time
                     return tuple(pt1_pos) # Return the start point of the zero-duration segment

                time_into_segment_seconds = (query_time_dt - pt1_time).total_seconds()
                alpha = time_into_segment_seconds / segment_duration_seconds

                current_pos = pt1_pos + alpha * (pt2_pos - pt1_pos)
                return tuple(current_pos)

        # Should ideally not be reached if query_time_dt is within start/end and timed_path is correctly built
        # As a fallback, return the last position if time is at or after the last point's time
        if query_time_dt >= self.timed_path[-1][0]:
             return self.timed_path[-1][1]

        return None # Should not happen with correct logic


def check_collisions(primary_drone, other_drones, safety_buffer=2.0, time_tolerance_seconds=1.0):
    """
    Checks for potential collisions between the primary drone and others
    by sampling points along their paths.
    """
    conflicts = []
    # Determine a suitable time step for collision checking.
    # Use the smallest time step among all drones to ensure fine-grained check.
    all_drones = [primary_drone] + other_drones
    # Initialize min_time_step to None, will update with the first valid step found
    min_time_step = None
    sim_start_time = None
    sim_end_time = None

    for drone in all_drones:
        if drone.timed_path:
            if sim_start_time is None or drone.start_time < sim_start_time:
                sim_start_time = drone.start_time
            if sim_end_time is None or drone.end_time > sim_end_time:
                sim_end_time = drone.end_time

            if len(drone.timed_path) > 1:
                # Calculate average time step for this drone's path segments
                total_duration = (drone.end_time - drone.start_time).total_seconds()
                num_intervals = len(drone.timed_path) - 1
                if total_duration > 0:
                    avg_segment_duration = total_duration / num_intervals
                    current_drone_time_step = timedelta(seconds=avg_segment_duration)
                    if min_time_step is None or current_drone_time_step < min_time_step:
                        min_time_step = current_drone_time_step

    if sim_start_time is None or sim_end_time is None or sim_start_time >= sim_end_time:
        print("Warning: Cannot perform collision check due to invalid simulation time window.")
        return conflicts # No valid time window to check

    # Use a small, fixed time step for checking, or base it on the min_time_step found
    # A fixed small step ensures consistent checking resolution regardless of path density
    # If no valid time steps were found (e.g., all drones stationary), use a default small step
    # Using a slightly smaller default step for static check for better chance of catching overlaps
    check_time_step = min_time_step if min_time_step is not None and min_time_step.total_seconds() > 0 else timedelta(seconds=0.05)


    current_check_time = sim_start_time
    while current_check_time <= sim_end_time:
        primary_pos = primary_drone.get_position_at_time(current_check_time)

        if primary_pos:
            primary_np = np.array(primary_pos, dtype=float)

            for other in other_drones:
                other_pos = other.get_position_at_time(current_check_time)

                if other_pos:
                    other_np = np.array(other_pos, dtype=float)
                    distance = np.linalg.norm(primary_np - other_np)

                    # Check for collision based on distance and time proximity (already at same time step)
                    if distance <= safety_buffer:
                        # Check if this conflict at this time for this pair has already been reported
                        # This prevents reporting the same conflict multiple times for adjacent time steps
                        already_reported = False
                        # Use a small time tolerance for checking if a conflict at this time has been reported
                        report_time_tolerance = timedelta(seconds=0.1) # Report conflicts within 0.1s of each other as the same
                        for c in conflicts:
                            time_diff = abs((c['time'] - current_check_time).total_seconds())
                            if c['other_drone'] == other.name and time_diff <= report_time_tolerance.total_seconds():
                                # Also check if the positions are very close, indicating it's the same event
                                if np.linalg.norm(np.array(c['primary_point']) - primary_np) < safety_buffer/2 and \
                                   np.linalg.norm(np.array(c['other_point']) - other_np) < safety_buffer/2:
                                    already_reported = True
                                    break


                        if not already_reported:
                             conflicts.append({
                                'primary_drone': primary_drone.name,
                                'other_drone': other.name,
                                'time': current_check_time,
                                'primary_point': primary_pos,
                                'other_point': other_pos,
                                'distance': distance
                            })

        current_check_time += check_time_step

    return conflicts


def visualize_missions(primary, others, conflicts=None): # Static 3D plot
    """Creates a static 3D visualization of drone missions and conflicts."""
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')

    all_drones_list = [primary] + others
    plot_colors = ['blue'] + ['green', 'purple', 'orange', 'cyan', 'magenta', 'brown'] * (len(others) // 6 + 1)

    for i, drone in enumerate(all_drones_list):
        drone_color = plot_colors[i % len(plot_colors)]

        # Plot path points
        path_pts = np.array([p for p in drone.spatial_points if p is not None and len(p)==3])
        if path_pts.size > 0:
            ax.plot(path_pts[:,0], path_pts[:,1], path_pts[:,2],
                    color=drone_color, linestyle='-', marker='o', markersize=2, alpha=0.7,
                    label=f'{drone.name} Path')

        # Plot waypoints and arrows
        wps_pts = np.array([wp for wp in drone.waypoints if wp is not None and len(wp)==3])
        if wps_pts.size > 0:
            ax.scatter(wps_pts[:,0], wps_pts[:,1], wps_pts[:,2],
                       color=drone_color, marker='D' if drone == primary else 's', s=50,
                       label=f'{drone.name} Waypoints')
            # Add arrows between waypoints
            for j in range(len(wps_pts) - 1):
                start_wp, end_wp = wps_pts[j], wps_pts[j+1]
                diff = end_wp - start_wp
                if np.linalg.norm(diff) > 1e-6: # Avoid drawing arrow for zero-length segments
                    ax.quiver(start_wp[0], start_wp[1], start_wp[2],
                              diff[0], diff[1], diff[2],
                              length=np.linalg.norm(diff)*0.8, # Arrow length slightly less than segment
                              arrow_length_ratio=0.1, # Adjust arrow head size
                              color=drone_color, normalize=True, alpha=0.5)


    # Plot conflicts
    if conflicts:
        conflict_points_data = [c['primary_point'] for c in conflicts if c['primary_point'] is not None and len(c['primary_point'])==3]
        if conflict_points_data:
            conflict_points = np.array(conflict_points_data)
            ax.scatter(conflict_points[:,0], conflict_points[:,1], conflict_points[:,2],
                       c='red', s=150, marker='X', label='Conflicts', depthshade=False, zorder=10)

    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    ax.set_title('3D Drone Missions and Conflicts (Static)')
    ax.legend(loc='upper left', bbox_to_anchor=(1.05, 1), borderaxespad=0.)
    ax.view_init(elev=20., azim=-35) # Adjust view angle
    ax.set_aspect('auto') # Adjust aspect ratio

    # Set plot limits based on all mission points
    all_coords_x = [p[0] for d in all_drones_list for t, coord_tuple in d.timed_path if coord_tuple for p in [coord_tuple] if p and len(p)==3]
    all_coords_y = [p[1] for d in all_drones_list for t, coord_tuple in d.timed_path if coord_tuple for p in [coord_tuple] if p and len(p)==3]
    all_coords_z = [p[2] for d in all_drones_list for t, coord_tuple in d.timed_path if coord_tuple for p in [coord_tuple] if p and len(p)==3]
    if all_coords_x and all_coords_y and all_coords_z:
        # Add padding to limits
        x_min, x_max = min(all_coords_x), max(all_coords_x)
        y_min, y_max = min(all_coords_y), max(all_coords_y)
        z_min, z_max = min(all_coords_z), max(all_coords_z)
        padding = 10 # meters
        ax.set_xlim(x_min - padding, x_max + padding)
        ax.set_ylim(y_min - padding, y_max + padding)
        ax.set_zlim(z_min - padding, z_max + padding)

    plt.tight_layout(rect=[0, 0, 0.85, 1]); # Adjust layout to make space for legend
    plt.show()


def animate_missions(primary_drone, other_drones, safety_buffer=2.0,
                     initial_sim_step_seconds=0.25,
                     movie_filename=None,
                     display_fps_target=30):
    """Animates drone missions and highlights conflicts."""

    fig = plt.figure(figsize=(14, 11))
    ax = fig.add_subplot(111, projection='3d')
    all_drones_list = [primary_drone] + other_drones

    # Determine simulation time window
    sim_start_time = min((d.start_time for d in all_drones_list if d.timed_path and d.start_time is not None), default=None)
    sim_end_time = max((d.end_time for d in all_drones_list if d.timed_path and d.end_time is not None), default=None)

    if sim_start_time is None or sim_end_time is None or sim_start_time >= sim_end_time:
        print("Animation cannot run: Invalid or empty simulation time window.")
        # Fallback to static visualization if animation is not possible
        static_conflicts = check_collisions(primary_drone, other_drones, safety_buffer)
        visualize_missions(primary_drone, other_drones, static_conflicts)
        return None

    drone_markers = []
    original_colors = {}
    plot_colors = ['blue'] + ['green', 'purple', 'orange', 'cyan', 'magenta', 'brown'] * (len(other_drones) // 6 + 1)

    # Plot static paths and waypoints
    for i, drone in enumerate(all_drones_list):
        color = plot_colors[i % len(plot_colors)]
        original_colors[drone.name] = color

        # Plot static path
        valid_spatial_points = [p for p in drone.spatial_points if p is not None and len(p) == 3]
        if valid_spatial_points:
            path_data = np.array(valid_spatial_points)
            if path_data.shape[0] > 0:
                ax.plot(path_data[:,0], path_data[:,1], path_data[:,2], linestyle=':', color=color, alpha=0.4, zorder=1)

        # Plot static waypoints
        valid_waypoints = [wp for wp in drone.waypoints if wp is not None and len(wp) == 3]
        if valid_waypoints:
            wp_data = np.array(valid_waypoints)
            if wp_data.shape[0] > 0:
                ax.scatter(wp_data[:,0], wp_data[:,1], wp_data[:,2], marker='o', color=color, s=20, alpha=0.6, zorder=2)

        # Create initial empty markers for the animated drones
        marker_style = 'D' if drone == primary_drone else 's'
        marker, = ax.plot([], [], [], marker=marker_style, markersize=8, color=color, zorder=5, label=drone.name)
        drone_markers.append(marker)

    # Add text indicators for time, collisions, and simulation speed
    time_text = ax.text2D(0.02, 0.95, '', transform=ax.transAxes, fontsize=12, bbox=dict(facecolor='white', alpha=0.7, boxstyle='round,pad=0.3'))
    collision_indicator_text = ax.text2D(0.5, 0.065, '', color='red', ha='center', transform=ax.transAxes, fontsize=14, weight='bold')
    speed_indicator_text = ax.text2D(0.98, 0.95, '', transform=ax.transAxes, fontsize=10, ha='right', bbox=dict(facecolor='white', alpha=0.7, boxstyle='round,pad=0.3'))

    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    ax.set_title('3D Drone Mission Animation'); ax.grid(True)
    ax.legend(loc='upper left', bbox_to_anchor=(1.05, 1), borderaxespad=0.)
    ax.view_init(elev=25, azim=-45) # Initial view angle
    ax.set_aspect('auto') # Adjust aspect ratio

    # Set plot limits based on all mission points
    all_coords_x = [p[0] for d in all_drones_list for t, coord_tuple in d.timed_path if coord_tuple for p in [coord_tuple] if p and len(p)==3]
    all_coords_y = [p[1] for d in all_drones_list for t, coord_tuple in d.timed_path if coord_tuple for p in [coord_tuple] if p and len(p)==3]
    all_coords_z = [p[2] for d in all_drones_list for t, coord_tuple in d.timed_path if coord_tuple for p in [coord_tuple] if p and len(p)==3]
    if all_coords_x and all_coords_y and all_coords_z:
        x_min, x_max = min(all_coords_x), max(all_coords_x)
        y_min, y_max = min(all_coords_y), max(all_coords_y)
        z_min, z_max = min(all_coords_z), max(all_coords_z)
        padding = 10 # meters
        ax.set_xlim(x_min - padding, x_max + padding)
        ax.set_ylim(y_min - padding, y_max + padding)
        ax.set_zlim(z_min - padding, z_max + padding)


    plt.subplots_adjust(left=0.05, right=0.75, bottom=0.15, top=0.95) # Adjust layout for buttons and legend

    # Animation state and controls
    ani_obj_storage = [None] # Use a list to store the animation object reference
    animation_state = {
        'paused': False,
        'current_sim_step_seconds': initial_sim_step_seconds,
        'current_sim_datetime': sim_start_time,
        'first_update_call': True,
        'last_collision_check_time': sim_start_time # Track last time collision was checked
    }
    MIN_SIM_STEP = 0.01; MAX_SIM_STEP = 5.0 # Define min/max simulation step for speed control
    COLLISION_CHECK_INTERVAL_SECONDS = 0.5 # How often to perform detailed collision check during animation

    def update_speed_display():
        """Updates the text displaying the current simulation step."""
        step = animation_state['current_sim_step_seconds']
        speed_indicator_text.set_text(f"Sim Step: {step:.2f}s")

    update_speed_display() # Initial display of speed

    # Define button callbacks BEFORE creating buttons
    def toggle_pause_resume_cb(event):
        """Callback for the Pause/Resume button."""
        if ani_obj_storage[0] is None: return
        animation_state['paused'] = not animation_state['paused']
        if animation_state['paused']:
            ani_obj_storage[0].pause()
            if 'pause' in button_refs: button_refs['pause'].label.set_text('Resume')
        else:
            ani_obj_storage[0].resume()
            if 'pause' in button_refs: button_refs['pause'].label.set_text('Pause')
        if fig.canvas: fig.canvas.draw_idle() # Redraw canvas to update button text

    def change_simulation_speed_cb(is_faster):
        """Callback for the Slower/Faster buttons."""
        current_step = animation_state['current_sim_step_seconds']
        factor = 1.33 if is_faster else 0.75
        new_step = current_step * factor
        animation_state['current_sim_step_seconds'] = max(MIN_SIM_STEP, min(new_step, MAX_SIM_STEP))
        update_speed_display()
        print(f"Sim step changed to: {animation_state['current_sim_step_seconds']:.3f} s/frame")

    # Initialize the list to store button widgets on the figure
    fig._button_widgets = []

    # Add control buttons (Pause/Resume, Slower, Faster)
    button_refs = {} # Store button references to prevent garbage collection
    ax_pause = fig.add_axes([0.20, 0.02, 0.10, 0.04]); btn_pause_widget = Button(ax_pause, 'Pause')
    btn_pause_widget.on_clicked(toggle_pause_resume_cb); fig._button_widgets.append(btn_pause_widget); button_refs['pause'] = btn_pause_widget

    ax_slow = fig.add_axes([0.32, 0.02, 0.12, 0.04]); btn_slow_widget = Button(ax_slow, 'Slower Sim')
    btn_slow_widget.on_clicked(lambda event: change_simulation_speed_cb(False)); fig._button_widgets.append(btn_slow_widget)

    ax_fast = fig.add_axes([0.46, 0.02, 0.12, 0.04]); btn_fast_widget = Button(ax_fast, 'Faster Sim')
    btn_fast_widget.on_clicked(lambda event: change_simulation_speed_cb(True)); fig._button_widgets.append(btn_fast_widget)


    def update(frame_index):
        """Update function for the animation."""
        t_start_update = time.perf_counter()

        # Advance simulation time if not paused
        if not animation_state['paused'] and not animation_state['first_update_call']:
            animation_state['current_sim_datetime'] += timedelta(seconds=animation_state['current_sim_step_seconds'])
        elif animation_state['first_update_call']:
             animation_state['first_update_call'] = False # Mark first call as done

        current_dt = animation_state['current_sim_datetime']

        # List of artists to update and return
        artists_to_return = [time_text, collision_indicator_text, speed_indicator_text] + drone_markers

        # Stop animation if simulation time exceeds end time
        if current_dt > sim_end_time + timedelta(seconds=animation_state['current_sim_step_seconds']):
            if ani_obj_storage[0] and ani_obj_storage[0].event_source and hasattr(ani_obj_storage[0].event_source, 'stop'):
                 ani_obj_storage[0].event_source.stop()
            time_text.set_text(f"End: {sim_end_time.strftime('%H:%M:%S.%f')[:-5]}")
            # Ensure markers are at their final positions
            for i, drone in enumerate(all_drones_list):
                 final_pos3d = drone.get_position_at_time(sim_end_time)
                 if final_pos3d and drone_markers[i].get_figure():
                     drone_markers[i].set_data_3d([final_pos3d[0]], [final_pos3d[1]], [final_pos3d[2]])
                     drone_markers[i].set_visible(True)
                 elif drone_markers[i].get_figure():
                     drone_markers[i].set_visible(False)
            if fig.canvas: fig.canvas.draw_idle() # Redraw to show final positions
            return artists_to_return

        # Update time display
        time_text.set_text(f"Time: {current_dt.strftime('%H:%M:%S.%f')[:-5]}")

        # Update drone positions and check for collisions
        current_positions = {}
        collision_this_frame = False
        colliding_pairs_text = []
        colliding_drone_indices = set() # Keep track of indices of colliding drones

        for i, drone in enumerate(all_drones_list):
            # Reset color before checking for collision in this frame
            if drone_markers[i].get_figure(): drone_markers[i].set_color(original_colors[drone.name])
            # Reset marker size
            if drone_markers[i].get_figure(): drone_markers[i].set_markersize(8)


            pos3d = drone.get_position_at_time(current_dt)
            if pos3d and drone_markers[i].get_figure():
                drone_markers[i].set_data_3d([pos3d[0]], [pos3d[1]], [pos3d[2]])
                drone_markers[i].set_visible(True)
                current_positions[drone.name] = (pos3d, i) # Store position and index
            elif drone_markers[i].get_figure():
                drone_markers[i].set_visible(False) # Hide marker if drone is not active

        # Check for collisions between the primary drone and others
        primary_pos_info = current_positions.get(primary_drone.name)
        if primary_pos_info:
            primary_pos3d, primary_idx = primary_pos_info
            primary_np = np.array(primary_pos3d)

            for other_drone in other_drones:
                other_pos_info = current_positions.get(other_drone.name)
                if other_pos_info:
                    other_pos3d, other_idx = other_pos_info
                    other_np = np.array(other_pos3d)
                    distance = np.linalg.norm(primary_np - other_np)

                    if distance < safety_buffer:
                        collision_this_frame = True
                        colliding_pairs_text.append(f"{primary_drone.name}↔{other_drone.name}")
                        colliding_drone_indices.add(primary_idx)
                        colliding_drone_indices.add(other_idx)
                        # Print collision details to console
                        print(f"🚨 Collision detected at {current_dt.strftime('%H:%M:%S.%f')[:-3]}: {primary_drone.name} and {other_drone.name} at distance {distance:.2f}m")


        # Highlight colliding drones by changing their marker color and size
        for idx in colliding_drone_indices:
             if drone_markers[idx].get_figure():
                 drone_markers[idx].set_color('red')
                 drone_markers[idx].set_markersize(12) # Make colliding markers larger


        # Update collision indicator text
        collision_indicator_text.set_text("COLLISION: " + " & ".join(colliding_pairs_text) if collision_this_frame else "")

        t_end_update = time.perf_counter()
        update_duration_ms = (t_end_update - t_start_update) * 1000
        # Optional: Print performance info periodically
        # if frame_index % 30 == 0 :
        #     display_interval_ms_val = max(1, int(1000.0 / display_fps_target))
        #     print(f"Frame {frame_index}: SimTime: {current_dt.strftime('%H:%M:%S')}, SimStep: {animation_state['current_sim_step_seconds']:.2f}s, Update: {update_duration_ms:.2f}ms, Target Display Interval: {display_interval_ms_val}ms")


        return artists_to_return # Return the updated artists

    # Calculate interval for animation frames based on target FPS
    display_interval_ms_val = max(1, int(1000.0 / display_fps_target))
    ani = FuncAnimation(fig, update, frames=itertools.count(),
                        interval=display_interval_ms_val, # Milliseconds between frames
                        blit=True, repeat=False, cache_frame_data=False) # Disable cache for potentially large data

    ani_obj_storage[0] = ani # Store animation object

    if movie_filename:
        try:
            print(f"Saving 3D animation to {movie_filename}...")
            # Set save_fps based on the initial simulation step for a realistic speed in the video
            save_fps = 1.0 / initial_sim_step_seconds
            # Note: Saving can be slow for complex animations or high FPS
            ani.save(movie_filename, writer='ffmpeg', fps=save_fps, dpi=150)
            print(f"3D Animation saved: {movie_filename}")
        except Exception as e:
            print(f"Error saving 3D animation: {e}. Ensure ffmpeg is installed/configured and in your system's PATH.")
            print("Attempting to display animation instead...")
            plt.show() # Show plot if saving fails
    else:
        plt.show() # Display animation

    return ani

def read_missions_from_file(filepath, base_time):
    """Reads drone mission data from a text file."""
    missions = []
    try:
        with open(filepath, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'): # Skip empty lines or comments
                    continue

                parts = line.split(';')
                if len(parts) < 4:
                    print(f"Warning: Skipping malformed line in {filepath}: {line}")
                    continue

                name = parts[0]
                try:
                    start_offset_seconds = float(parts[1])
                    duration_seconds = float(parts[2])
                except ValueError:
                    print(f"Warning: Skipping line with invalid time data in {filepath}: {line}")
                    continue

                start_time = base_time + timedelta(seconds=start_offset_seconds)
                end_time = start_time + timedelta(seconds=duration_seconds)
                time_window = (start_time, end_time)

                waypoints_str = parts[3:]
                waypoints = []
                for wp_str in waypoints_str:
                    coords = wp_str.split(',')
                    if len(coords) == 3:
                        try:
                            waypoint = (float(coords[0]), float(coords[1]), float(coords[2]))
                            waypoints.append(waypoint)
                        except ValueError:
                            print(f"Warning: Skipping invalid waypoint format in line: {line} -> {wp_str}")
                            # Continue processing other waypoints in the same line if possible
                            pass # Skip this waypoint but try the next one
                    else:
                         print(f"Warning: Skipping waypoint with incorrect number of coordinates in line: {line} -> {wp_str}")


                if not waypoints:
                    print(f"Warning: Skipping mission '{name}' with no valid waypoints.")
                    continue

                missions.append(DroneMission(waypoints, time_window, name=name))

    except FileNotFoundError:
        print(f"Error: Mission file not found at {filepath}")
        return None
    except Exception as e:
        print(f"An error occurred while reading mission file {filepath}: {e}")
        return None

    return missions


# Example usage with reading missions from a file
if __name__ == "__main__":
    # Define a common base time for all missions
    base_time = datetime(2025, 5, 1, 12, 0, 0)

    # Define a safety buffer (e.g., 2.5 meters)
    safety_margin = 2.5

    # --- Generate a sample mission file if it doesn't exist ---
    mission_file_path = "random_collision_missions.txt" #----> CHANGED THIS DIRECTORY random_missions.txt
    # Removed the call to generate_mission_file from here.
    # You should run generate_missions.py separately first.
    if not os.path.exists(mission_file_path):
        print(f"Error: Mission file '{mission_file_path}' not found.")
        print("Please run 'generate_missions.py' first to create the mission file.")
        exit() # Exit if the mission file is not found

    # --- Read missions from the file ---
    print(f"--- Reading Missions from {mission_file_path} ---")
    all_missions = read_missions_from_file(mission_file_path, base_time)

    if all_missions is None or not all_missions:
        print("No missions loaded. Exiting.")
    else:
        # Assume the first mission in the file is the primary drone
        primary_drone = all_missions[0]
        other_drones_list = all_missions[1:]

        print(f"\nLoaded {len(all_missions)} missions.")
        print(f"Primary Drone: {primary_drone.name}")
        print(f"Other Drones: {[d.name for d in other_drones_list]}")
        print(f"Safety Buffer: {safety_margin}m")


        # --- Static Collision Check ---
        print("\n--- Static 3D Collision Check Results ---")
        # Using a smaller time tolerance for static check to be more precise
        static_conflicts = check_collisions(primary_drone, other_drones_list,
                                            safety_buffer=safety_margin, time_tolerance_seconds=0.5)
        if static_conflicts:
            print(f"🚨 STATIC 3D CHECK: Found {len(static_conflicts)} potential conflicts!")
            # Print details for the first few conflicts to avoid spamming console
            for i, c in enumerate(static_conflicts[:10]):
                 print(f"  Conflict {i+1}: Time: {c['time'].strftime('%H:%M:%S.%f')[:-3]}, Drones: {c['primary_drone']} & {c['other_drone']}, Dist: {c['distance']:.2f}m")
            if len(static_conflicts) > 10:
                 print(f"  ... {len(static_conflicts) - 10} more conflicts not shown.")
        else:
            print("✅ STATIC 3D CHECK: No conflicts detected by batch check.")

        # --- Animation ---
        print("\n--- Starting 3D Animation (close plot window to exit) ---")
        # Adjust initial_sim_step_seconds for animation speed
        animation_object = animate_missions(primary_drone, other_drones_list,
                                            safety_buffer=safety_margin,
                                            initial_sim_step_seconds=0.5, # Can adjust this for animation speed
                                            display_fps_target=30)


