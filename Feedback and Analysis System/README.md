# Feedback and Analysis System

Develop a feedback module that evaluates the scanned map (e.g., coverage, accuracy, or completeness). \
Display results or metrics in RViz2 or log them for further analysis. ✔️\
Optionally integrate interactive feedback—like highlighting unscanned areas or suggesting re-exploration paths. ⏳\

This module listens to the robot’s 2D map and turns it into simple metrics.

## How values of cells are interpreted in Rviz
```
-1 → unknown (not scanned yet)
0 → free (traversable) 
1..100 → occupied (obstacle)
```

## What we count
```
total = all cells in the grid
known = cells that are not -1
free = cells that are 0
occ (occupied) = cells that are > 0
```

## What we compute
```
coverage % = known / total * 100
free % = free / total * 100
occupied % = occ / total * 100
```
coverage is the percentage of the map that the robot has observed (cells that are not -1), whether free or occupied \
free is the percentage of the map that is observed and empty/traversable (cells that are 0) \
occ (occupied) is the percentage of the map that is observed and not free (cells that are 1–100), i.e. objects, walls, or obstacles \

## What we publish
```
text line (for logs): coverage=.. free=.. occ=..
```

# Run the Feedback Module steps

## 1. Start the isaac simulation
### In terminal One
```
conda activate env_isaaclab
cd ~/isaac-go2-ros2-isaacsim-4.5
python isaac-go2-ros2.py
```

## 2. Start Rviz perception module
### In terminal Two
```
conda activate env_isaaclab
ros2 launch navigation_runner perception.launch.py
```
### In terminal Three
```
ros2 launch navigation_runner rviz.launch.py
```

## 3. Start map feedback module
### In terminal Four
```
cd ~/isaac-go2-ros2-isaacsim-4.5
source install/setup.bash
python3 -m map_feedback.map_feedback_node --ros-args -r /map:=/occupancy_map/occupancy_map_2D
```