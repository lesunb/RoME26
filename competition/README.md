# RoME26 MRS Competition

## Overview

 The TurtleRescue Challenge is a fast-paced robotics competition where teams program a TurtleBot4 to explore a small indoor “disaster zone,” locate victim markers, and return safely to base with a mission report.

TurtleRescue gives students a hands-on introduction to autonomous robotics, mission design, and search-and-rescue principles using accessible tools and real-world-inspired tasks.

  - Goals: Program a TurtleBot4 to explore, detect victims, and return to base.
  - Duration: 2-hour session + practice time.
  - Teams: Up to 4 teams per match cycle.
  - Robots: 4× TurtleBot4.
  - Difficulty: Introductory → intermediate.

## Mission Description

Each team's robot must autonomously:

1. Start at the base station
2. Map the unknown arena (SLAM)
3. Locate victims
4. Record their positions
5. Return to base
6. Deliver a final “Mission Report”

Victims are marked as red cones on the map, scoring is based on the number of detections, the total area explored, and the time to complete the mission (tiebreaker). 

## Environment

The mission will take place inside a controlled environment, where robots must navigate autonomously and perform tasks for the personnel present. 

For testing we recommend using the turtlebot4 warehouse map, which is already included alongside your default Turtlebot4 installation.

<img width="474" height="240" alt="image" src="https://github.com/user-attachments/assets/eec8bc1b-8ac4-4db1-86ff-6305beccb33c" />

## Implementation

Missions must be implemented using ROS2 Humble and the Turtlebot4 Lite robots. For the implementation, we recommend using control structures to simplify your design, such as Behavior Trees ([BT.CPP](https://www.behaviortree.dev/docs/3.8/), [py_trees](https://py-trees.readthedocs.io/)), or state machines ([SMACH](https://docs.ros.org/en/rolling/p/smach/), [FlexBE](http://flexbe.github.io/)).

## Example Mission: Lab Samples Logistics

In the Lab Samples mission, a robot must autonomously pick up and deliver samples inside a hospital Scenario. Namely:

1. Start at the base station
2. Navigate to a nurse position
3. Identify and authenticate the nurse before picking up a sample
4. Navigate to the laboratory
5. Identify and authenticate the lab personnel
6. Deliver the sample
7. Return to base

We provide an example implementation of this mission using the BT.CPP framework which you can use for your own mission.
