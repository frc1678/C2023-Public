# C2023
[![CI](https://github.com/frc1678/C2023-Public/actions/workflows/main.yml/badge.svg)](https://github.com/frc1678/C2023-Public/actions/workflows/main.yml)

![Robot Image](/Images/Robot.jpg)

---
Code for Team 1678's 2023 robot, Tangerine Tumbler.

## Highlights
  - Field-Relative Swerve Drive 
  - 3 Game Piece Autonomous Modes
  - Gyro-Based Autonomous Dock and Engage
  - Sequenced State Transitions for Scoring
  - Fork Climb

## Major Package Functions
- [`com.team1678.frc2023`](src/main/java/com/team1678/frc2023/)
	
	Contains central robot functions specific to Tangerine Tumbler.  Robot control originates from the [`Robot`](/src/main/java/com/team1678/frc2023/Robot.java) class.

- [`com.team1678.frc2023.auto`](src/main/java/com/team1678/frc2023/auto)
	
	Handles generation, selection, and execution of autonomous routines.

- [`com.team1678.frc2023.auto.actions`](src/main/java/com/team1678/frc2023/auto/actions/)
	
	Contains all actions used during autonomous.  All actions must implement the [`Action`](src/main/java/com/team1678/frc2023/auto/actions/Action.java) interface.  

- [`com.team1678.frc2023.auto.modes`](src/main/java/com/team1678/frc2023/auto/modes/)
	
	Contains all autonomous modes.  Modes are named according to starting location (CC for cable chain side, middle for center, nothing for flat side), number of game pieces scored (One/Two/Three), and if it engages.

 - [`com.team1678.frc2023.controlboard`](src/main/java/com/team1678/frc2023/controlboard/)
	
	Handles polling driver and operator inputs from two [`Xbox Controllers`](src/main/java/com/team1678/frc2023/controlboard/CustomXboxController.java).

 - [`com.team1678.frc2023.loops`](src/main/java/com/team1678/frc2023/loops/)
	
	Contains code for loops, which are run periodically to update Subsystems.  [`Loops`](src/main/java/com/team1678/frc2023/loops/Loop.java) are managed and run by [`Loopers`](src/main/java/com/team1678/frc2023/loops/Looper.java).  The [`Robot`](/src/main/java/com/team1678/frc2023/Robot.java) class contains three loops: one for enabled operation, one for IO while disabled, and one for CSV logging.

- [`com.team1678.frc2023.shuffleboard`](src/main/java/com/team1678/frc2023/shuffleboard/)
	
	Contains layouts for reporting telemetry to the Shuffleboard.  Entries are organized into [`Tabs`](src/main/java/com/team1678/frc2023/shuffleboard/tabs/), which extend the [`ShuffleboardTabBase`](src/main/java/com/team1678/frc2023/shuffleboard/ShuffleboardTabBase.java).

- [`com.team1678.frc2023.states`](src/main/java/com/team1678/frc2023/states/)

	Contains one class, [`SuperstructureGoal`](src/main/java/com/team1678/frc2023/states/SuperstructureGoal.java), which holds preset setpoints for the arm, elevator, and wrist.

- [`com.team1678.frc2023.subsystems`](src/main/java/com/team1678/frc2023/subsystems/)

	Contains code for subsystems, with one singleton implementation per subsystem.  Subsystems extend the [`Subsystem`](src/main/java/com/team1678/frc2023/subsystems/Subsystem.java) abstract class. Each subsystem's logic is contained in an enabled loop, a read periodic inputs method, and a write periodic outputs method, which are called by the [`SubsystemManager`](src/main/java/com/team1678/frc2023/SubsystemManager.java) class.  


- [`com.team1678.lib.logger`](src/main/java/com/team1678/lib/logger/)

	Contains code for logging fields tagged by the [`Log`](src/main/java/com/team1678/lib/logger/Log.java) annotation to CSV files.


