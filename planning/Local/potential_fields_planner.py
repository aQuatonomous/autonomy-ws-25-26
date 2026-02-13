#!/usr/bin/env python3
"""
Potential Fields Path Planner:
Computes collision-free paths using potential fields algorithm.

Inputs:
    - Complete entity list with goals (from global Planner)
    - Current position

Outputs (for Mavros):
    - Path waypoints to goal(s)
    - Velocity setpoints for Mavros integration    

"""


import numpy as np

# ============================================================================
# TUNABLE PARAMETERS - Adjust these to modify planner behavior
# ============================================================================

# Potential Field Gains
K_ATT = 1.8          # Attractive gain - higher = stronger pull toward goals
K_REP = 140.0        # Repulsive gain - higher = stronger push from obstacles
D_INFLUENCE = 10.0   # Influence distance for obstacles (ft)

# Movement Control
MAX_VELOCITY = 2.0   # Maximum velocity (ft/s) - reduce for smoother motion
STEP_SIZE = 0.25     # Base step size for gradient descent
MAX_STEP = 0.5       # Hard cap on max motion per iteration
GRADIENT_DAMPING = 0.3  # Damping factor for gradient (higher = smoother, slower)

# Gate Approach
GATE_APPROACH_DIST = 35.0  # Distance to start gate alignment (ft)
GATE_STRENGTH = 0.6        # Gate alignment force strength
GATE_OFFSET = 0.8          # Offset bias toward red buoy side

# Convergence Criteria
GOAL_THRESHOLD = 2.0       # Distance to consider goal reached (ft)
MIN_GRADIENT = 1e-3        # Minimum gradient to escape local minima
LOCAL_MIN_NUDGE = 0.2      # Random nudge distance when stuck

# Path Smoothing
SMOOTH_WINDOW = 5          # Window size for path smoothing
SMOOTH_ITERATIONS = 2      # Number of smoothing passes
MIN_POINT_SPACING = 0.1    # Minimum distance between path points (ft)

# Velocity Computation
DT = 0.1                   # Time step for velocity computation (seconds)
VELOCITY_SMOOTH_WINDOW = 3 # Window for velocity smoothing

# ============================================================================


class PotentialFieldsPlanner:
    def __init__(self, resolution=0.5):
        """
        Initialize planner with configurable resolution.
        
        Args:
            resolution: Grid resolution for potential field computation
        """
        self.resolution = resolution
        
        # Will be set when computing paths
        self.map_width = None
        self.map_height = None
        self.X = None
        self.Y = None
        
        # Store controllable parameters as instance variables
        self.k_att = K_ATT
        self.k_rep = K_REP
        self.d_influence = D_INFLUENCE
        self.max_velocity = MAX_VELOCITY
        self.step_size = STEP_SIZE
        self.max_step = MAX_STEP
        self.gradient_damping = GRADIENT_DAMPING
        self.gate_approach_dist = GATE_APPROACH_DIST
        self.gate_strength = GATE_STRENGTH
        self.gate_offset = GATE_OFFSET
        self.goal_threshold = GOAL_THRESHOLD
        self.min_gradient = MIN_GRADIENT
        self.local_min_nudge = LOCAL_MIN_NUDGE
        self.smooth_window = SMOOTH_WINDOW
        self.smooth_iterations = SMOOTH_ITERATIONS
        self.min_point_spacing = MIN_POINT_SPACING
        self.dt = DT
        self.velocity_smooth_window = VELOCITY_SMOOTH_WINDOW
    
    def initialize_grid(self, map_width, map_height):
        """Initialize the planning grid based on map dimensions"""
        self.map_width = map_width
        self.map_height = map_height
        self.x = np.arange(0, map_width, self.resolution)
        self.y = np.arange(0, map_height, self.resolution)
        self.X, self.Y = np.meshgrid(self.x, self.y)
        
    def compute_attractive_potential(self, goal_x, goal_y):
        """Compute attractive potential field toward goal"""
        if self.X is None:
            raise ValueError("Grid not initialized. Call initialize_grid() first.")
        dist = np.sqrt((self.X - goal_x)**2 + (self.Y - goal_y)**2)
        return 0.5 * self.k_att * dist**2
    
    def compute_repulsive_potential(self, obstacle_x, obstacle_y):
        """Compute repulsive potential field from obstacle"""
        if self.X is None:
            raise ValueError("Grid not initialized. Call initialize_grid() first.")
        dist = np.sqrt((self.X - obstacle_x)**2 + (self.Y - obstacle_y)**2)
        potential = np.zeros_like(dist)
        
        # Apply strong repulsive force within influence distance
        mask = dist <= self.d_influence
        # Use exponential repulsion for stronger effect
        potential[mask] = self.k_rep * np.exp(-dist[mask] / 2.0)
        
        return potential
    
    def compute_total_potential(self, goal, obstacles):
        """
        Compute total potential field
        
        Args:
            goal: Tuple (x, y) of goal position
            obstacles: List of tuples [(x1, y1), (x2, y2), ...] of obstacle positions
        """
        if self.X is None:
            raise ValueError("Grid not initialized. Call initialize_grid() first.")
            
        goal_x, goal_y = goal
        U_total = self.compute_attractive_potential(goal_x, goal_y)
        
        # Add repulsive potentials from all obstacles
        for obs_x, obs_y in obstacles:
            U_total += self.compute_repulsive_potential(obs_x, obs_y)
        
        return U_total
    
    def gradient_descent_path(self, start_x, start_y, goal_x, goal_y, obstacles, 
                              red_pos=None, green_pos=None, max_steps=4000):
        """
        Compute path using gradient descent on potential field
        
        Args:
            start_x, start_y: Starting position
            goal_x, goal_y: Goal position
            obstacles: List of obstacle positions [(x1, y1), ...]
            red_pos: Optional red buoy position for gate alignment
            green_pos: Optional green buoy position for gate alignment
            max_steps: Maximum iterations
            
        Returns:
            path: numpy array of waypoints [(x1, y1), (x2, y2), ...]
        """
        path = [(start_x, start_y)]
        x, y = start_x, start_y
        
        # Gate direction vector (red to green) if provided
        gate_normal = None
        if red_pos is not None and green_pos is not None:
            gate_vec = np.array([green_pos[0] - red_pos[0], green_pos[1] - red_pos[1]])
            gate_normal = np.array([-gate_vec[1], gate_vec[0]])  # Perpendicular (left normal)
            gate_normal = gate_normal / np.linalg.norm(gate_normal)
        
        for step in range(max_steps):
            # Check if goal reached
            dist_to_goal = np.sqrt((x - goal_x)**2 + (y - goal_y)**2)
            if dist_to_goal < self.goal_threshold:
                path.append((goal_x, goal_y))
                break
            
            # Compute gradient at current position
            # Attractive gradient (toward goal)
            grad_att_x = self.k_att * (x - goal_x)
            grad_att_y = self.k_att * (y - goal_y)
            
            # Repulsive gradient (away from obstacles)
            grad_rep_x = 0
            grad_rep_y = 0
            
            for obs_x, obs_y in obstacles:
                dist = np.sqrt((x - obs_x)**2 + (y - obs_y)**2)
                if dist <= self.d_influence and dist > 0.01:
                    # Exponential repulsion gradient
                    repulsion_strength = self.k_rep * np.exp(-dist / 2.0) / (2.0 * dist)
                    grad_rep_x -= repulsion_strength * (x - obs_x)
                    grad_rep_y -= repulsion_strength * (y - obs_y)
                elif dist <= 0.01:
                    # Very close to obstacle - strong push away
                    angle = np.random.uniform(0, 2*np.pi)
                    grad_rep_x -= 1000 * np.cos(angle)
                    grad_rep_y -= 1000 * np.sin(angle)
            
            # Add gate alignment force when approaching goal (if gate provided)
            grad_gate_x = 0
            grad_gate_y = 0
            if gate_normal is not None and dist_to_goal < self.gate_approach_dist:
                # Bias toward correct approach (offset to left/red side)
                offset = gate_normal * self.gate_offset
                target_x = goal_x + offset[0]
                target_y = goal_y + offset[1]
                # Stronger pull when closer to gate
                gate_strength = self.gate_strength * (1.0 - dist_to_goal / self.gate_approach_dist)
                grad_gate_x = gate_strength * (x - target_x)
                grad_gate_y = gate_strength * (y - target_y)
            
            # Total gradient
            grad_x = grad_att_x + grad_rep_x + grad_gate_x
            grad_y = grad_att_y + grad_rep_y + grad_gate_y
            
            # Adaptive step size based on gradient magnitude
            grad_norm = np.hypot(grad_x, grad_y)

            if grad_norm < self.min_gradient:
                # Almost at an equilibrium – if we're close to goal, stop
                if dist_to_goal < self.goal_threshold:
                    path.append((goal_x, goal_y))
                    break
                # Else tiny random nudge to escape a local minimum
                angle = np.random.uniform(0, 2*np.pi)
                x += self.local_min_nudge * np.cos(angle)
                y += self.local_min_nudge * np.sin(angle)
            else:
                # Smoother step control
                base_step = self.step_size
                step = base_step / (1.0 + self.gradient_damping * grad_norm)
                # Hard cap on max motion per iteration
                step = min(step, self.max_step / grad_norm)

                x -= step * grad_x
                y -= step * grad_y

            
            # Keep within bounds
            if self.map_width is not None and self.map_height is not None:
                x = np.clip(x, 0, self.map_width)
                y = np.clip(y, 0, self.map_height)
            
            # Only add point if it moved enough (reduce redundant points)
            if len(path) == 0 or np.sqrt((x - path[-1][0])**2 + (y - path[-1][1])**2) > self.min_point_spacing:
                path.append((x, y))
        
        return np.array(path)
    
    def smooth_path(self, path, window=None, iterations=None):
        """Smooth path using simple moving average"""
        if window is None:
            window = self.smooth_window
        if iterations is None:
            iterations = self.smooth_iterations
            
        smoothed = path.copy()
        for _ in range(iterations):
            for i in range(window, len(smoothed) - window):
                smoothed[i] = np.mean(smoothed[i-window:i+window+1], axis=0)
        # Ensure endpoints (goals) remain fixed
        return smoothed
    
    def compute_velocities(self, path):
        """
        Compute velocity setpoints from path for Mavros integration
        
        Args:
            path: numpy array of waypoints [(x1, y1), ...]
            
        Returns:
            velocities: numpy array of velocity vectors [(vx1, vy1), ...]
            speeds: numpy array of speed magnitudes [v1, v2, ...]
        """
        if len(path) < 2:
            return np.array([]), np.array([])
        
        # Compute velocities using finite differences
        velocities = np.diff(path, axis=0) / self.dt
        
        # Smooth velocities
        if len(velocities) > self.velocity_smooth_window * 2:
            smoothed_vel = velocities.copy()
            for i in range(self.velocity_smooth_window, len(smoothed_vel) - self.velocity_smooth_window):
                smoothed_vel[i] = np.mean(
                    smoothed_vel[i-self.velocity_smooth_window:i+self.velocity_smooth_window+1], 
                    axis=0
                )
            velocities = smoothed_vel
        
        # Compute speeds
        speeds = np.linalg.norm(velocities, axis=1)
        
        # Clip to max velocity
        mask = speeds > self.max_velocity
        if np.any(mask):
            velocities[mask] = velocities[mask] / speeds[mask, np.newaxis] * self.max_velocity
            speeds[mask] = self.max_velocity
        
        # Add final velocity (same as last)
        velocities = np.vstack([velocities, velocities[-1]])
        speeds = np.append(speeds, speeds[-1])
        
        return velocities, speeds
    
    def plan_path(self, start, goal, obstacles, gates=None, map_bounds=None):
        """
        Plan a path from start to goal avoiding obstacles
        
        Args:
            start: Tuple (x, y) start position
            goal: Tuple (x, y) goal position
            obstacles: List of obstacle positions [(x1, y1), ...]
            gates: Optional list of gate pairs [((red_x, red_y), (green_x, green_y)), ...]
            map_bounds: Optional tuple (width, height) for map bounds
            
        Returns:
            dict with keys:
                'path': numpy array of waypoints
                'velocities': numpy array of velocity setpoints
                'speeds': numpy array of speed magnitudes
        """
        # Initialize grid if bounds provided
        if map_bounds is not None:
            self.initialize_grid(map_bounds[0], map_bounds[1])
        
        # Extract gate positions if provided
        red_pos = None
        green_pos = None
        if gates is not None and len(gates) > 0:
            red_pos, green_pos = gates[0]  # Use first gate
        
        # Compute path
        path = self.gradient_descent_path(
            start[0], start[1], goal[0], goal[1], 
            obstacles, red_pos, green_pos
        )
        
        # Smooth path
        smoothed_path = self.smooth_path(path)
        
        # Compute velocities
        velocities, speeds = self.compute_velocities(smoothed_path)
        
        return {
            'path': smoothed_path,
            'velocities': velocities,
            'speeds': speeds
        }
    
    def plan_multi_goal_path(self, start, goals, obstacles, gates=None, map_bounds=None):
        """
        Plan a path through multiple goals sequentially
        
        Args:
            start: Tuple (x, y) start position
            goals: List of goal positions [(x1, y1), (x2, y2), ...]
            obstacles: List of obstacle positions [(x1, y1), ...]
            gates: Optional list of gate pairs [((red_x, red_y), (green_x, green_y)), ...]
            map_bounds: Optional tuple (width, height) for map bounds
            
        Returns:
            dict with keys:
                'path': numpy array of waypoints
                'velocities': numpy array of velocity setpoints
                'speeds': numpy array of speed magnitudes
                'segment_indices': list of indices where each segment ends
        """
        # Initialize grid if bounds provided
        if map_bounds is not None:
            self.initialize_grid(map_bounds[0], map_bounds[1])
        
        full_path = []
        segment_indices = []
        current_pos = start
        
        for i, goal in enumerate(goals):
            # Get gate positions if available
            red_pos = None
            green_pos = None
            if gates is not None and i < len(gates):
                red_pos, green_pos = gates[i]
            
            # Compute segment
            segment = self.gradient_descent_path(
                current_pos[0], current_pos[1], 
                goal[0], goal[1], 
                obstacles, red_pos, green_pos
            )
            
            # Append to full path
            if len(full_path) == 0:
                full_path = segment
            else:
                full_path = np.vstack([full_path, segment[1:]])
            
            segment_indices.append(len(full_path) - 1)
            current_pos = goal
        
        # Smooth the path
        smoothed_path = self.smooth_path(full_path)
        
        # Compute velocities
        velocities, speeds = self.compute_velocities(smoothed_path)
        
        return {
            'path': smoothed_path,
            'velocities': velocities,
            'speeds': speeds,
            'segment_indices': segment_indices
        }

    def plan_local_path(self, start, visible_goals, obstacles, gates=None, 
                        map_bounds=None, horizon_limit=3):
        """
        Plan path using rolling horizon - only to nearest visible goals.
        Optimized for continuous replanning with partial information.
        
        This is preferred for real-time navigation with dynamic entity discovery
        as it computes faster paths to nearby goals rather than the full course.
        
        Args:
            start: Tuple (x, y) current position
            visible_goals: List of currently visible goal positions
            obstacles: List of currently visible obstacles
            gates: Optional list of visible gate pairs
            map_bounds: Optional tuple (width, height)
            horizon_limit: Max number of goals to plan toward (default 3)
            
        Returns:
            dict with keys:
                'path': numpy array of waypoints to nearest goals
                'velocities': numpy array of velocity setpoints
                'speeds': numpy array of speed magnitudes
                'planned_goals': number of goals included in plan
        """
        if not visible_goals:
            # No goals visible - return empty path
            return {
                'path': np.array([start]),
                'velocities': np.array([[0.0, 0.0]]),
                'speeds': np.array([0.0]),
                'planned_goals': 0
            }
        
        # Sort goals by distance and take nearest ones
        start_arr = np.array(start)
        goal_distances = [np.linalg.norm(np.array(g) - start_arr) for g in visible_goals]
        sorted_indices = np.argsort(goal_distances)
        
        # Plan to nearest N goals
        planning_goals = [visible_goals[i] for i in sorted_indices[:horizon_limit]]
        planning_gates = None
        if gates:
            planning_gates = [gates[i] for i in sorted_indices[:horizon_limit] if i < len(gates)]
        
        # Use multi-goal planner for the limited set
        result = self.plan_multi_goal_path(
            start=start,
            goals=planning_goals,
            obstacles=obstacles,
            gates=planning_gates,
            map_bounds=map_bounds
        )
        
        result['planned_goals'] = len(planning_goals)
        return result

