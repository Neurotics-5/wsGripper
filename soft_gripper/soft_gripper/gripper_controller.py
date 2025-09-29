import numpy as np
import time

class GripperController:
    def __init__(self, dynamixel_client, motor_ids, config):
        self.client = dynamixel_client
        self.motor_ids = motor_ids
        self.open_positions = [config['open_positions'][mid] for mid in motor_ids]

        self.min_position = {str(mid): config['min_position_rad'][mid] for mid in motor_ids}
        self.max_position = {str(mid): config['max_position_rad'][mid] for mid in motor_ids}

        self.current_control_mode = 'position'
        self.goal_positions = None  # Do not predefine a goal that triggers movement

        self.l1 = config.get('linkBase', 0.1)
        self.l2 = config.get('linkTact', 0.1) 
        self.l3 = config.get('linkTip', 0.1)  

        self.grip = ''

        self.pid_kp = 0.01  # Proportional gain (tune experimentally)
        self.pid_ki = 0.0
        self.pid_kd = 0.0

        self.pid_integral = 0.0
        self.pid_last_error = 0.0

        self.last_positions = np.zeros(len(self.motor_ids))
        self.last_load = np.zeros(len(self.motor_ids))

        self.contact_threshold = 0.3 # Example: threshold at which contact is considered made
        self.max_load_threshold = 1.0  # Example: stall-level load
        self.tip_tolerance_rad = np.deg2rad(20)  # Small follow-up motion for tip fingers

        # --- tactile force-hold control (closed-loop on force) ---
        self.force_hold_active = False
        self.force_target_N = 0.0
        self.force_kp = 0.015   # start conservative; tune on hardware
        self.force_ki = 0.003   # small integral to remove bias
        self.force_int = 0.0
        self.force_theta = None  # parallel angle we regulate (radians)
        self.force_max_N = 15.0  # absolute max force (safety)

        self.contact_enable_N = 2.0   # enable I-term after this force
        self.integrator_beta = 0.2    # back-calculation factor for anti-windup (simple)

    def connect(self):
        self.client.connect()
        self.client.set_torque_enabled(self.motor_ids, False)  # Always start with motors disabled

    def disconnect(self):
        self.client.disconnect()

    def set_torque_enabled(self, enabled: bool):
        self.client.set_torque_enabled(self.motor_ids, enabled)

    def open(self):
        self.stop_force_hold()  # ensure loop is off
        self.goal_positions = np.copy(self.open_positions)
        self.client.write_desired_pos(self.motor_ids, self.goal_positions)


    def _theta_to_targets(self, theta):
        """Map the parallel 'theta' to joint targets (keeps parallel kinematics)."""
        return {
            1: float(np.clip(theta + np.pi, self.min_position["1"], self.max_position["1"])),
            2: float(np.clip(-theta + np.pi, self.min_position["2"], self.max_position["2"])),
            3: float(np.clip(-theta + np.pi, self.min_position["3"], self.max_position["3"])),
            4: float(np.clip(theta + np.pi, self.min_position["4"], self.max_position["4"])),
        }
    
    def start_parallel_force_hold(self, target_N):
        """Begin closed-loop parallel force regulation using tactile sensors."""
        tgt = float(np.clip(abs(float(target_N)), 1.0, self.force_max_N))  # obey 1..15 N and |.| 
        self.force_target_N = tgt
        self.force_int = 0.0
        self.grip = 'ParallelForceHold'

        # Initialize theta from current pose so we "continue" the parallel motion
        positions = self.read_current_positions()
        # joints 1 and 4 map to +theta + pi  (see _theta_to_targets)
        theta_a = positions[0] - np.pi
        theta_b = positions[3] - np.pi
        theta_max = (np.pi / 2.0) - 0.01
        self.force_theta = float(np.clip(np.mean([theta_a, theta_b]), 0.0, theta_max))


        # Sync goal to current-theta targets so updates are smooth
        self.move_to_positions(self._theta_to_targets(self.force_theta))
        self.force_hold_active = True

    def stop_force_hold(self):
        """Exit force-hold mode (does not automatically open)."""
        self.force_hold_active = False
        self.grip = ''

    def update_force_hold(self, measured_force_N, dt=0.01):
        """
        One control step of force-hold. Call at ~100 Hz from the ROS node with the
        averaged absolute force from the left/right tactile sensors.
        """
        if not self.force_hold_active:
            return

        meas = abs(float(measured_force_N))

        # Safety: stop if we exceed the max by a small margin
        if meas > (self.force_max_N + 0.5):
            print("[force_hold] Safety stop: measured force exceeded limit.")
            self.stop_force_hold()
            return

        err = self.force_target_N - meas
        if abs(err) < 0.05:  # small deadband
            err = 0.0

        # ----- Conditional integrator -----
        # Only integrate after firm contact to avoid windup while closing on air.
        if meas >= self.contact_enable_N and err != 0.0:
            self.force_int += err * float(dt)
        # clamp integral
        self.force_int = float(np.clip(self.force_int, -5.0, 5.0))

        # P + I (negative sign closes when force too low)
        u = (self.force_kp * err) + (self.force_ki * self.force_int)
        dtheta = float(np.clip(-u, -0.01, 0.01))

        # Propose new theta and clamp to geometry
        theta_unclamped = (self.force_theta if self.force_theta is not None else 0.0) + dtheta
        theta_max = (np.pi / 2.0) - 0.01
        new_theta = float(np.clip(theta_unclamped, 0.0, theta_max))

        # ----- Simple back-calculation anti-windup -----
        # If we hit saturation, bleed the integrator toward a value that would
        # have produced the clamped output.
        if new_theta != theta_unclamped:
            sat_error = theta_unclamped - new_theta  # >0 means we tried to move past the limit
            # push the integral a bit opposite to the saturation direction
            self.force_int -= self.integrator_beta * sat_error

        self.force_theta = new_theta
        self.move_to_positions(self._theta_to_targets(self.force_theta))
        
    def parallel_grip(self, width):
        """
        Perform a parallel grip to achieve the specified width (in mm) between fingertips.
        Assumes:
        - Motors 1,2 = Finger A (base, tip)
        - Motors 3,4 = Finger B (base, tip), mirrored
        """
        self.grip = 'Parallel'

        y_dis = width / 2.0  # One side of the total width

        if y_dis > self.l1:
            print(f"[parallel_grip] Width too large: half-width {y_dis} mm exceeds l1 = {self.l1}")
            return

        try:
            theta = np.arcsin(y_dis / self.l1)
        except ValueError:
            print("[parallel_grip] Invalid width: arcsin input out of range")
            return

        # Calculate joint targets (adjusted to actual joint ranges in config)
        target_positions = {
            1: np.clip(theta + np.pi, self.min_position["1"], self.max_position["1"]),
            2: np.clip(-theta + np.pi, self.min_position["2"], self.max_position["2"]),
            3: np.clip(-theta + np.pi, self.min_position["3"], self.max_position["3"]),
            4: np.clip(theta + np.pi, self.min_position["4"], self.max_position["4"]),
        }

        self.move_to_positions(target_positions)



    def pinch_grip(self, state: int):
        """
        Instant pinch grip position:
        - state = 1 → open (tips: 210°, 150°)
        - state = 0 → closed (tips: 180°)
        Base joints (1 and 4) always at 180°.
        """
        self.grip = 'Pinch'

        if state == 1:
            # Open configuration
            target_positions = {
                1: np.deg2rad(200),        # base A
                2: np.deg2rad(200),        # tip A
                3: np.deg2rad(160),        # base B
                4: np.deg2rad(160),        # tip B
            }
        elif state == 0:
            # Closed configuration
            target_positions = {
                1: np.deg2rad(200),        # base A
                2: np.deg2rad(140),        # tip A
                3: np.deg2rad(160),        # base B
                4: np.deg2rad(220),        # tip B
            }
        else:
            print(f"[pinch_grip] Invalid state: {state}. Use 1=open or 0=close.")
            return

        # Apply joint limits
        for mid in target_positions:
            target_positions[mid] = np.clip(
                target_positions[mid],
                self.min_position[str(mid)],
                self.max_position[str(mid)]
            )

        self.move_to_positions(target_positions)
        self.current_positions = target_positions

    def read_current_positions(self):
        positions, _, _, _ = self.client.read_pos_vel_load_temp()
        return positions

    def update(self):
        positions, velocities, load, temperature = self.client.read_pos_vel_load_temp()

        # Compute derived values (only if grip mode needs it)
        if self.grip in ('Pinch', 'Parallel', 'ParallelForceHold'):
            x_pos = self.l1 * np.cos(positions[1]) + self.l2 * np.cos(positions[1] + positions[2])
            width = self.l1 * np.cos(positions[1])
        elif self.grip == 'Form':
            x_pos = 100.0
            width = 100.0
        else:
            x_pos = -1.0
            width = -1.0

        return {
            'positions': positions,
            'load': load,
            'grip': self.grip,
            'width': width,
            'x_pos': x_pos,
            'temperature': temperature
        }


    def is_goal_reached(self, tolerance=0.05):
        if self.goal_positions is None:
            return True

        current_pos = self.read_current_positions()
        return np.allclose(current_pos, self.goal_positions, atol=tolerance)


    def move_to_positions(self, target_dict, speed_dict=None):
        if self.goal_positions is None:
            self.goal_positions = self.read_current_positions()

        pos_array = np.copy(self.goal_positions)
        speed_array = np.full(len(self.motor_ids), 600)  # default speed

        for i, mid in enumerate(self.motor_ids):
            if mid in target_dict:
                pos_array[i] = target_dict[mid]
            if speed_dict and mid in speed_dict:
                speed_array[i] = speed_dict[mid]

        self.client.write_profile_velocity(self.motor_ids, speed_array)
        self.goal_positions = pos_array
        self.client.write_desired_pos(self.motor_ids, pos_array)
    
    def reboot_all_motors(self):
        self.client.reboot_motors(self.motor_ids)
