# F1TENTH Autonomous Racing System

This repository contains a complete F1TENTH autonomous racing system with localization, path planning, and path following capabilities. The system supports both real car deployment and simulation environments.

## System Architecture

The autonomous racing system consists of four main components that must be launched in sequence:

1. **Localization** - Particle filter for robot pose estimation
2. **F1TENTH System** - Hardware interface (real car) or Simulation bridge 
3. **Path Planning** - Lattice planner for trajectory generation
4. **Path Following** - Controller to execute planned trajectories

## Installation

### Clone Repository

This repository uses submodules, so clone with recursive flag:

```bash
# Clone with submodules
git clone --recursive <repository-url>
cd f1tenth_all_js

# If already cloned without submodules, initialize them:
git submodule update --init --recursive
```

### Update Repository

To pull latest changes including submodules:

```bash
# Pull main repository and all submodules
git pull --recurse-submodules

# Alternative method:
git pull
git submodule update --remote --recursive
```

### Build Workspace

```bash
# Build the workspace
colcon build
source install/setup.bash
```

## Usage Instructions

### Step 1: Launch Localization (Particle Filter)

Choose the appropriate localization mode:

#### For Real Car (Default)
```bash
ros2 launch particle_filter_cpp localize_launch.py
```

#### For SLAM Map Mode (Real Car)
```bash
ros2 launch particle_filter_cpp localize_slam_launch.py
```

#### For Simulation
```bash
ros2 launch particle_filter_cpp localize_sim_launch.py
```

### Step 2: Launch F1TENTH System or Simulation

#### For Real Car
```bash
ros2 launch f1tenth_stack bringup_launch.py
```

#### For Simulation
```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

### Step 3: Set Initial Pose

After launching the localization system:

1. Open RViz (should launch automatically with localization)
2. Click the **"2D Pose Estimate"** button in RViz toolbar
3. Click and drag on the map to set the robot's initial position and orientation
4. Verify that the particle cloud converges around the robot's actual position

### Step 4: Launch Path Planner (Lattice Planner)

#### For Real Car (Default - SLAM map mode)
```bash
ros2 launch lattice_planner_pkg lattice_planner.launch.py
```

#### For Simulation
```bash
ros2 launch lattice_planner_pkg lattice_planner.launch.py sim_mode:=true
```

### Step 5: Launch Path Follower

#### For Real Car
```bash
ros2 launch path_follower_pkg path_follower.launch.py sim_mode:=false
```

#### For Simulation
```bash
ros2 launch path_follower_pkg path_follower.launch.py sim_mode:=true
```

## Launch Parameters

### Particle Filter Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `map_name` | Auto-detected | Map file name (without .yaml extension) |
| `scan_topic` | `/scan` | Laser scan topic |
| `odom_topic` | `/odom` (real), `/ego_racecar/odom` (sim) | Odometry topic |
| `use_rviz` | `true` | Launch RViz visualization |
| `use_sim_time` | `false` | Use simulation time |

### Lattice Planner Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `sim_mode` | `false` | Use simulation topics if true |
| `use_sim_time` | `false` | Use simulation time |

### Path Follower Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `sim_mode` | `true` | Use simulation topics if true |

## Topic Remapping

### Real Car Mode
- Odometry: `/pf/pose/odom` (from particle filter)
- Laser: `/scan`
- Drive commands: `/drive`

### Simulation Mode  
- Odometry: `/ego_racecar/odom` (from F1TENTH Gym)
- Laser: `/scan`
- Drive commands: `/drive`

## Configuration Files

### Localization Configs
- `src/particle_filter_cpp/config/localize.yaml` - Real car settings
- `src/particle_filter_cpp/config/localize_slam.yaml` - SLAM map settings  
- `src/particle_filter_cpp/config/localize_sim.yaml` - Simulation settings

### Planning & Control Configs
- `src/lattice_planner_f1tenth/config/planner_config.yaml` - Path planning parameters
- `src/lattice_path_follower_f1tenth/config/path_follower_config.yaml` - Control parameters

### Maps
- `src/particle_filter_cpp/maps/` - Map files (.png/.pgm and .yaml)
- `src/f1tenth_gym_ros/maps/` - Simulation maps

## Example Launch Sequences

### Real Car with SLAM Map
```bash
# Terminal 1: Localization with SLAM map
ros2 launch particle_filter_cpp localize_slam_launch.py

# Terminal 2: F1TENTH hardware interface  
ros2 launch f1tenth_stack bringup_launch.py

# Terminal 3: Set initial pose in RViz, then launch planner
ros2 launch lattice_planner_pkg lattice_planner.launch.py

# Terminal 4: Launch path follower
ros2 launch path_follower_pkg path_follower.launch.py sim_mode:=false
```

### Simulation Mode
```bash
# Terminal 1: Localization for simulation
ros2 launch particle_filter_cpp localize_sim_launch.py

# Terminal 2: F1TENTH Gym simulation
ros2 launch f1tenth_gym_ros gym_bridge_launch.py

# Terminal 3: Set initial pose in RViz, then launch planner  
ros2 launch lattice_planner_pkg lattice_planner.launch.py sim_mode:=true

# Terminal 4: Launch path follower
ros2 launch path_follower_pkg path_follower.launch.py sim_mode:=true
```

## Troubleshooting

### Common Issues

1. **Particle filter not converging**: Ensure initial pose is set correctly using RViz 2D Pose Estimate
2. **No laser data**: Check that scan topics match between localization and hardware/simulation
3. **Planning failures**: Verify that localization is working and pose estimates are stable
4. **Simulation not starting**: Ensure F1TENTH Gym dependencies are installed

### Debug Topics

Monitor these topics for debugging:
- `/scan` - Laser scan data
- `/odom` or `/ego_racecar/odom` - Odometry data  
- `/pf/pose/odom` - Particle filter pose estimate
- `/planned_path` - Generated trajectories
- `/drive` - Drive commands

## Dependencies

- ROS 2 (Humble/Iron)
- Navigation2 stack
- F1TENTH Gym (for simulation)
- RViz2
- Custom packages included in this workspace

## Package Structure

```
src/
├── f1tenth_base_setup/         # F1TENTH hardware drivers and configs
├── f1tenth_gym_ros/            # F1TENTH Gym simulation bridge  
├── particle_filter_cpp/        # Localization system
├── lattice_planner_f1tenth/    # Path planning algorithms
└── lattice_path_follower_f1tenth/ # Path following control
```