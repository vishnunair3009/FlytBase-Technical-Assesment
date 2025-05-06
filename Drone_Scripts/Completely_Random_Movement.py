# COMPLETELY RANDOM MOVEMENT 

import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime, timedelta
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button
import time
import itertools # For itertools.count()

# --- Configuration ---
SIM_BOUNDS = {'x': (-50, 50), 'y': (-50, 50), 'z': (0, 30)} # Simulation volume (min, max)
INITIAL_SPEED_RANGE = (1.0, 5.0) # meters per second
VELOCITY_PERTURBATION = 0.5 # Maximum random change added to velocity components per second
SAFETY_BUFFER = 2.0 # meters - Minimum distance allowed between drones
NUM_DRONES = 5
ANIMATION_DT = 0.05 # Seconds per animation frame (affects simulation granularity and speed)
DISPLAY_FPS_TARGET = 30 # Target frames per second for display
MOVIE_FILENAME = None # Set to a string like 'random_collision_animation.mp4' to save video

# --- Random Drone Class ---
class RandomDrone:
    def __init__(self, name, sim_bounds, safety_buffer=2.0):
        self.name = name
        self.sim_bounds = sim_bounds
        self.safety_buffer = safety_buffer

        # Initialize random position within bounds
        self.position = np.array([
            np.random.uniform(sim_bounds['x'][0], sim_bounds['x'][1]),
            np.random.uniform(sim_bounds['y'][0], sim_bounds['y'][1]),
            np.random.uniform(sim_bounds['z'][0], sim_bounds['z'][1])
        ], dtype=float)

        # Initialize random velocity within speed range
        initial_speed = np.random.uniform(INITIAL_SPEED_RANGE[0], INITIAL_SPEED_RANGE[1])
        random_direction = np.random.uniform(-1.0, 1.0, 3)
        # Normalize direction and scale by speed
        direction_norm = np.linalg.norm(random_direction)
        if direction_norm > 1e-6: # Avoid division by zero
            self.velocity = (random_direction / direction_norm) * initial_speed
        else: # If random_direction was (0,0,0), pick a default direction
             self.velocity = np.array([0.0, 0.0, 1.0]) * initial_speed # Example: move up

    def update(self, dt):
        """Updates the drone's position based on velocity and applies boundary logic."""
        # Add random perturbation to velocity
        perturbation = np.random.uniform(-VELOCITY_PERTURBATION, VELOCITY_PERTURBATION, 3) * dt # Scale perturbation by dt
        self.velocity += perturbation

        # Optional: Clamp velocity magnitude to prevent speeds from getting extreme
        current_speed = np.linalg.norm(self.velocity)
        if current_speed > INITIAL_SPEED_RANGE[1] * 2: # Don't let speed double max initial speed
            self.velocity = (self.velocity / current_speed) * (INITIAL_SPEED_RANGE[1] * 2)
        elif current_speed < INITIAL_SPEED_RANGE[0] / 2 and current_speed > 1e-6: # Don't let speed halve min initial speed
             self.velocity = (self.velocity / current_speed) * (INITIAL_SPEED_RANGE[0] / 2)
        elif current_speed < 1e-6: # If speed is near zero, give it a little push
             self.velocity = np.random.uniform(-1.0, 1.0, 3) * INITIAL_SPEED_RANGE[0] / 2


        # Update position
        self.position += self.velocity * dt

        # Boundary bouncing logic
        for i in range(3): # Iterate through x, y, z dimensions
            min_bound, max_bound = (self.sim_bounds['x'][0], self.sim_bounds['x'][1]) if i == 0 else \
                                   (self.sim_bounds['y'][0], self.sim_bounds['y'][1]) if i == 1 else \
                                   (self.sim_bounds['z'][0], self.sim_bounds['z'][1])

            # Check lower bound
            if self.position[i] < min_bound:
                self.position[i] = min_bound # Place exactly on boundary
                self.velocity[i] *= -1 # Reverse velocity component
                # Optional: Add a small random bounce angle change
                self.velocity += np.random.uniform(-0.1, 0.1, 3) * np.sign(self.velocity[i]) # Small push away

            # Check upper bound
            elif self.position[i] > max_bound:
                self.position[i] = max_bound # Place exactly on boundary
                self.velocity[i] *= -1 # Reverse velocity component
                 # Optional: Add a small random bounce angle change
                self.velocity += np.random.uniform(-0.1, 0.1, 3) * np.sign(self.velocity[i]) # Small push away


            # Ensure velocity is still reasonable after bouncing/perturbation
            # Re-clamping might be needed here if the push above adds too much speed
            current_speed = np.linalg.norm(self.velocity)
            max_clamped_speed = INITIAL_SPEED_RANGE[1] * 2
            if current_speed > max_clamped_speed:
                 self.velocity = (self.velocity / current_speed) * max_clamped_speed


    def get_position(self):
        """Returns the current position."""
        return tuple(self.position)

# --- Simulation and Visualization ---

def run_random_simulation(num_drones, sim_bounds, safety_buffer, anim_dt, display_fps_target, movie_filename=None):
    """Sets up and runs the random drone simulation with animation."""

    drones = [RandomDrone(f"Drone {i+1}", sim_bounds, safety_buffer) for i in range(num_drones)]

    if not drones:
        print("No drones to simulate.")
        return

    fig = plt.figure(figsize=(14, 11))
    ax = fig.add_subplot(111, projection='3d')

    drone_markers = []
    original_colors = {}
    # Use distinct colors
    plot_colors = plt.cm.get_cmap('tab10', num_drones) # Get a colormap with enough colors

    # Create initial markers for the animated drones
    for i, drone in enumerate(drones):
        color = plot_colors(i)
        original_colors[drone.name] = color

        marker, = ax.plot([], [], [], marker='o', markersize=8, color=color, zorder=5, label=drone.name)
        drone_markers.append(marker)

    # Set plot limits based on simulation bounds
    ax.set_xlim(sim_bounds['x'])
    ax.set_ylim(sim_bounds['y'])
    ax.set_zlim(sim_bounds['z'])

    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    ax.set_title('3D Random Drone Movement and Collisions')
    ax.grid(True)
    ax.legend(loc='upper left', bbox_to_anchor=(1.05, 1), borderaxespad=0.)
    ax.view_init(elev=25, azim=-45) # Initial view angle
    ax.set_aspect('auto')

    # Add text indicators for time, collisions, and simulation speed
    # Use arbitrary start time for display purposes
    sim_start_time = datetime.now().replace(microsecond=0)
    animation_state = {
        'paused': False,
        'current_sim_datetime': sim_start_time,
        'total_sim_time_elapsed': 0.0
    }

    time_text = ax.text2D(0.02, 0.95, '', transform=ax.transAxes, fontsize=12, bbox=dict(facecolor='white', alpha=0.7, boxstyle='round,pad=0.3'))
    collision_indicator_text = ax.text2D(0.5, 0.065, '', color='red', ha='center', transform=ax.transAxes, fontsize=14, weight='bold')
    # Speed indicator is less relevant here as speed is controlled by ANIMATION_DT and matplotlib's interval

    plt.subplots_adjust(left=0.05, right=0.75, bottom=0.05, top=0.95) # Adjust layout for buttons and legend

    # Animation state and controls
    ani_obj_storage = [None] # Use a list to store the animation object reference

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

    # Initialize the list to store button widgets on the figure
    fig._button_widgets = [] # Prevent garbage collection

    # Add control buttons (Pause/Resume)
    button_refs = {} # Store button references
    ax_pause = fig.add_axes([0.20, 0.01, 0.10, 0.04]); btn_pause_widget = Button(ax_pause, 'Pause')
    btn_pause_widget.on_clicked(toggle_pause_resume_cb); fig._button_widgets.append(btn_pause_widget); button_refs['pause'] = btn_pause_widget


    def update(frame_index):
        """Update function for the animation - advances simulation time and updates positions/collisions."""
        t_start_update = time.perf_counter()

        # Advance simulation time if not paused
        if not animation_state['paused']:
            # Update total simulation time
            animation_state['total_sim_time_elapsed'] += anim_dt
            # Update display time (using a reference start time + elapsed)
            animation_state['current_sim_datetime'] = sim_start_time + timedelta(seconds=animation_state['total_sim_time_elapsed'])

            # Update drone positions
            for drone in drones:
                drone.update(anim_dt) # Pass the animation time step as simulation time step

        current_dt = animation_state['current_sim_datetime']

        # List of artists to update and return
        artists_to_return = [time_text, collision_indicator_text] + drone_markers

        # Update time display
        time_text.set_text(f"Sim Time: {animation_state['total_sim_time_elapsed']:.2f} s\nWall Clock: {current_dt.strftime('%H:%M:%S')}")

        # Get current positions for collision check and plotting
        current_positions = [(drone.get_position(), i, drone.name) for i, drone in enumerate(drones)]

        # Reset colors and sizes before collision check
        for i in range(len(drones)):
            if drone_markers[i].get_figure(): # Check if marker object is valid
                 drone_markers[i].set_color(original_colors[drones[i].name])
                 drone_markers[i].set_markersize(8)


        # Collision Check for this frame
        collision_this_frame = False
        colliding_pairs_text = []
        colliding_drone_indices = set() # Keep track of indices of colliding drones

        # Iterate through unique pairs of drones
        for i in range(len(drones)):
            for j in range(i + 1, len(drones)): # Only check each pair once
                pos1, idx1, name1 = current_positions[i]
                pos2, idx2, name2 = current_positions[j]

                pos1_np = np.array(pos1)
                pos2_np = np.array(pos2)
                distance = np.linalg.norm(pos1_np - pos2_np)

                if distance <= safety_buffer:
                    collision_this_frame = True
                    colliding_pairs_text.append(f"{name1}↔{name2}")
                    colliding_drone_indices.add(idx1)
                    colliding_drone_indices.add(idx2)
                    # Print collision details to console
                    # Using total_sim_time_elapsed for more precise timing in console
                    print(f"🚨 Collision detected at sim time {animation_state['total_sim_time_elapsed']:.2f} s: {name1} and {name2} at distance {distance:.2f}m")

        # Highlight colliding drones by changing their marker color and size
        for idx in colliding_drone_indices:
             if drone_markers[idx].get_figure():
                 drone_markers[idx].set_color('red')
                 drone_markers[idx].set_markersize(12) # Make colliding markers larger


        # Update drone marker positions for plotting
        for i, (pos, idx, name) in enumerate(current_positions):
             if drone_markers[idx].get_figure():
                 drone_markers[idx].set_data_3d([pos[0]], [pos[1]], [pos[2]])
                 drone_markers[idx].set_visible(True)


        # Update collision indicator text
        collision_indicator_text.set_text("COLLISION: " + " & ".join(colliding_pairs_text) if collision_this_frame else "")

        t_end_update = time.perf_counter()
        update_duration_ms = (t_end_update - t_start_update) * 1000
        # Optional: Print performance info periodically
        # if frame_index % (display_fps_target * 5) == 0: # Print every 5 seconds of wall clock time
        #      print(f"Frame {frame_index}: Sim Time: {animation_state['total_sim_time_elapsed']:.2f}s, Update: {update_duration_ms:.2f}ms")


        return artists_to_return # Return the updated artists

    # Calculate interval for animation frames based on target FPS
    # Matplotlib's interval is in milliseconds
    display_interval_ms_val = max(1, int(1000.0 / display_fps_target))

    # Create the animation
    # frames=itertools.count() runs the animation indefinitely
    ani = FuncAnimation(fig, update, frames=itertools.count(),
                        interval=display_interval_ms_val,
                        blit=True, repeat=False, cache_frame_data=False)

    ani_obj_storage[0] = ani # Store animation object

    if movie_filename:
        try:
            print(f"Saving 3D animation to {movie_filename}...")
            # Setting a high fps for saving makes the random walk look smoother/faster in the video
            # Adjust save_fps relative to ANIMATION_DT and desired video speed
            save_fps = display_fps_target # Save at the same rate we display
            ani.save(movie_filename, writer='ffmpeg', fps=save_fps, dpi=150)
            print(f"3D Animation saved: {movie_filename}")
        except Exception as e:
            print(f"Error saving 3D animation: {e}. Ensure ffmpeg is installed/configured and in your system's PATH.")
            print("Attempting to display animation instead...")
            plt.show() # Show plot if saving fails
    else:
        plt.show() # Display animation

# --- Main Execution ---
if __name__ == "__main__":
    print("Starting random drone simulation...")
    run_random_simulation(
        num_drones=NUM_DRONES,
        sim_bounds=SIM_BOUNDS,
        safety_buffer=SAFETY_BUFFER,
        anim_dt=ANIMATION_DT,
        display_fps_target=DISPLAY_FPS_TARGET,
        movie_filename=MOVIE_FILENAME # Set to None to just display, or a filename string to save
    )
    print("Simulation finished.")