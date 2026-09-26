# Yug - Reactive Behaviors

Current work area for the LiDAR and autonomous reactive behaviors.

## Responsibilities

- LiDAR scan processing
- Forward behavior
- Asymmetric obstacle avoidance
- Symmetric obstacle escape behavior
- Random turning after approximately 1 ft of forward movement
- Behavior priority/integration with existing group nodes

## Current integration work

- Confirm TurtleBot 4 LaserScan topic
- Determine front LiDAR sectors
- Tune symmetric vs. asymmetric obstacle detection
- Connect distance tracker event to random-turn behavior
- Coordinate velocity command arbitration with keyboard and bumper nodes
- Test behavior transitions in Gazebo

## Required autonomous priority

1. Escape
2. Avoid
3. Random turn
4. Forward

The complete integrated priority will also place bumper and keyboard control
above these autonomous behaviors.
