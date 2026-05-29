# FTC Pedro Skeleton

A production-ready FTC robot codebase built on **PedroPathing** for autonomous path following and **FTCLib** for command-based structure. Designed to stay maintainable across a full season and easy to hand off between programmers.

---

## Table of Contents

1. [Framework Overview](#framework-overview)
2. [Project Structure](#project-structure)
3. [Hardware Requirements](#hardware-requirements)
4. [Getting Started](#getting-started)
5. [Tuning Guide](#tuning-guide)
6. [Maintenance Guidelines](#maintenance-guidelines)
7. [Contribution Rules](#contribution-rules)

---

## Framework Overview

### PedroPathing (Autonomous Path Following)

[PedroPathing](https://github.com/Pedro-Pathing/PedroPathing) is a vector-based, closed-loop path follower. At every control loop tick it projects the robot's position onto a Bézier curve and applies four simultaneous correction vectors:

| Vector | Corrects |
|---|---|
| **Drive** | Velocity along the path tangent — keeps the robot at the right speed |
| **Translational** | X/Y error from the closest point on the path |
| **Heading** | Rotational error from the desired heading at that point |
| **Centripetal** | Drift caused by centripetal acceleration on curves |

Each vector uses a **dual-PIDF**: a primary PIDF for large errors, a secondary PIDF that activates close to the target. This resolves the oscillation-vs-precision tradeoff common in single-loop systems.

Paths are Bézier curves (line, quadratic, or cubic) chained into a `PathChain`. Heading along each segment can be interpolated linearly, held constant, or follow the path tangent.

### FTCLib (Command-Based Architecture)

[FTCLib](https://github.com/FTCLib/FTCLib) provides a command-based model adapted from WPILib (FRC). The three building blocks are:

- **Subsystem** — owns a set of hardware actuators. Its `periodic()` method runs every loop tick via the scheduler.
- **Command** — a single, discrete robot action. It declares which subsystems it requires; the scheduler prevents two commands from fighting over the same hardware.
- **CommandScheduler** — orchestrates everything inside `CommandOpMode.run()`: calls `periodic()` on all subsystems, then advances active commands through their lifecycle (`initialize → execute → isFinished → end`).

The result is that both TeleOp and Autonomous logic decompose into small, independently testable pieces rather than a single monolithic `loop()`.

### Why this combination?

| Concern | Solved by |
|---|---|
| Precise autonomous movement | PedroPathing Bézier follower + dual-PIDF correction |
| Readable, modular TeleOp | FTCLib command-based scheduler |
| Consistent localization across both modes | PedroPathing localizer (always running in `periodic()`) |
| In-field tuning without redeployment | `Tuning` OpMode + Panels Dashboard |

---

## Project Structure

```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
│
├── pedroPathing/
│   ├── Constants.java          # Central config: FollowerConstants, PathConstraints,
│   │                           # and createFollower() factory used everywhere
│   └── Tuning.java             # All built-in PedroPathing tuning OpModes (do not edit)
│
├── subsystems/
│   └── DriveSubsystem.java     # Follower wrapped as an FTCLib SubsystemBase
│   # Add: ArmSubsystem.java, IntakeSubsystem.java, ...
│
├── commands/
│   ├── TeleOpDriveCommand.java # Default drive command: gamepad → DriveSubsystem
│   └── FollowPathCommand.java  # Wraps a Path or PathChain as an FTCLib command
│   # Add one Command file per discrete robot action
│
└── opmodes/
    ├── teleop/
    │   └── TeleOpMode.java     # Main driver-controlled OpMode
    └── auto/
        └── AutoSample.java     # Example autonomous OpMode
        # Add: AutoSpecimen.java, AutoPark.java, ...
```

### Dependency graph

```
OpMode (CommandOpMode)
  └── CommandScheduler
        ├── DriveSubsystem.periodic()  →  Follower.update()  (PedroPathing)
        ├── OtherSubsystem.periodic()  →  hardware actuators
        └── Command.execute()          →  subsystem method calls
```

---

## Hardware Requirements

### Drive Motors

Four mecanum-wheel motors configured in the robot's hardware map with these exact names:

| Config name | Position |
|---|---|
| `leftFront` | Front-left |
| `rightFront` | Front-right |
| `leftBack` | Rear-left |
| `rightBack` | Rear-right |

Motor directions depend on your wiring. Verify them with the `Localization Test` OpMode before any path work.

### Localization (Odometry)

PedroPathing requires a localizer that produces a continuous pose estimate (X, Y, heading). Choose **one** and configure it in `Constants.createFollower()` via `FollowerBuilder`.

| Localizer | Accuracy | Hardware | Notes |
|---|---|---|---|
| **Three-wheel odometry** | Highest | 3 dead-wheel pods + motor/port encoders | No IMU needed; recommended for competition |
| **Two-wheel + IMU** | Good | 2 dead-wheel pods + Control Hub IMU | IMU heading drifts slightly over long matches |
| **Drive encoder + IMU** | Baseline | Control Hub IMU only | Wheel slip degrades accuracy; prototyping only |
| **goBILDA Pinpoint** | Excellent | Pinpoint odometry computer (I2C) | Easiest hardware setup, plug-and-play |
| **SparkFun OTOS** | Excellent | OTOS optical sensor (I2C) | Optical tracking; unaffected by wheel slip |

**Three-wheel odometry** requires specifying each pod's position (in inches from robot center) in `FollowerConstants`:

```java
// Example — measure from your physical robot
followerConstants.leftEncoderPose   = new Pose(-6.5,  0,    Math.PI / 2);
followerConstants.rightEncoderPose  = new Pose( 6.5,  0,    Math.PI / 2);
followerConstants.strafeEncoderPose = new Pose( 0,   -3.5,  0);
```

Run the **Offsets Tuner** in the `Tuning` OpMode to verify these values after measuring.

### Control Hub

- Keep the IMU correctly oriented if using any IMU-backed localizer.
- The `Tuning` OpMode requires the **Panels** companion app on a device connected to the Control Hub's Wi-Fi network.

---

## Getting Started

### 1. Verify the dependency

`TeamCode/build.gradle` applies `build.dependencies.gradle`, which pulls in PedroPathing and FTCLib. Sync Gradle and confirm there are no resolution errors.

### 2. Configure hardware names

In `Constants.java`, set motor names and your chosen localizer inside `createFollower()`. Adjust `FollowerConstants` fields (mass, velocity limits) to match your robot after tuning.

### 3. Tune the robot

Follow the [Tuning Guide](#tuning-guide) below in order. Do not skip steps — each builds on values measured in the previous one.

### 4. Validate localization

Run `Localization Test`. Drive the robot through the full field and confirm the Panels Dashboard pose tracks within ~1 inch and ~2° of real position.

### 5. Run a smoke test

Deploy `AutoSample.java` on the actual field to confirm end-to-end path following before writing competition autos.

---

## Tuning Guide

All tuning is done through the `Tuning` OpMode (group "Pedro Pathing" on the Driver Station). A laptop or phone running the **Panels** app must be on the same Wi-Fi as the Control Hub.

Run in this exact order:

```
Localization
  1. Localization Test                        Confirm odometry tracks correctly (visual check)
  2. Offsets Tuner                            Correct dead-wheel X/Y offset from robot center
  3. Forward Tuner                            Calibrate forward ticks-to-inches multiplier
  4. Lateral Tuner                            Calibrate strafe ticks-to-inches multiplier
  5. Turn Tuner                               Calibrate rotation ticks-to-radians multiplier

Automatic  (robot physics characterization)
  6. Forward Velocity Tuner                   Measure max forward velocity
  7. Lateral Velocity Tuner                   Measure max lateral velocity
  8. Forward Zero Power Acceleration Tuner    Measure forward deceleration
  9. Lateral Zero Power Acceleration Tuner    Measure lateral deceleration
 10. Predictive Braking Tuner                 (Optional) Fit braking curve for sharper stops

Manual  (PIDF tuning)
 11. Heading Tuner         Hold robot still; twist it manually and watch correction
 12. Translational Tuner   Hold robot on a path; push it laterally and watch correction
 13. Drive Tuner           Robot runs forward/back; observe velocity tracking
 14. Centripetal Tuner     Robot runs a curve; observe lateral drift

Tests  (end-to-end validation — run these after any constant change)
 15. Line                  All PIDFs active, straight back-and-forth
 16. Triangle              Multi-segment path with heading changes
 17. Circle                Full centripetal/heading stress test
```

Write all final values into `Constants.java`. Never hard-code them in OpMode or Command files.

---

## Maintenance Guidelines

These rules exist so any team member can read, modify, and test any part of the codebase without understanding all of it.

### One subsystem per hardware group

Every physical mechanism gets exactly one `SubsystemBase` class. No OpMode or Command ever calls `hardwareMap.get()` directly — hardware initialization belongs in the subsystem constructor only.

```java
// Correct
intake.extend();

// Wrong — hardware access leaking into a command
((Servo) hardwareMap.get("intakeServo")).setPosition(1.0);
```

### One command per action

Each discrete robot action is one `CommandBase` subclass. Commands do not know about other commands — compose them in the OpMode using `SequentialCommandGroup`, `ParallelCommandGroup`, etc.

```java
// Correct — composed at the OpMode level
schedule(new SequentialCommandGroup(
    new FollowPathCommand(drive, scorePath, true),
    new ScoreSampleCommand(arm, intake)
));

// Wrong — one command doing everything
// execute() { drive forward, raise arm, score, drive back }
```

### Constants live in constants classes

Every tunable value (PIDF gain, servo position, path pose, speed limit) is a `static` field in a dedicated constants class. Magic numbers inside OpMode or Command files are not allowed.

```java
// Correct
path.setConstantHeadingInterpolation(RobotConstants.SCORING_HEADING);

// Wrong
path.setConstantHeadingInterpolation(1.047);
```

### No logic in OpModes

OpModes are wiring only: instantiate subsystems, build paths, schedule commands. Conditional logic and math belong in Commands or helper classes.

### Build paths once, in `initialize()`

`Path` and `PathChain` construction is allocation-heavy. Build all paths during `initialize()`, not inside a loop or a command's `execute()`.

```java
@Override
public void initialize() {
    scoringPath = drive.getFollower().pathBuilder()
            .addPath(...)
            .build();                    // built once

    schedule(new FollowPathCommand(drive, scoringPath, true));
}
```

### Keep `periodic()` lean

`DriveSubsystem.periodic()` calls `follower.update()` every loop. No slow operations (I2C reads, string formatting, file I/O) belong in any `periodic()` method. Telemetry output belongs in commands.

### Naming conventions

| Element | Convention | Example |
|---|---|---|
| Subsystem class | `*Subsystem` | `ArmSubsystem` |
| Command class | verb + noun | `ScoreSampleCommand` |
| TeleOp OpMode | `TeleOp*` | `TeleOpMode` |
| Auto OpMode | `Auto*` | `AutoSpecimen` |
| Constant field | `UPPER_SNAKE_CASE` | `ARM_SCORE_ANGLE_DEG` |
| Path variable | descriptive noun | `scoreFirstSamplePath` |

### Autonomous OpMode template

Every auto OpMode follows this pattern:

```java
@Autonomous(name = "AutoXxx", group = "OpModes")
public class AutoXxx extends CommandOpMode {

    @Override
    public void initialize() {
        // 1. Instantiate subsystems
        DriveSubsystem drive = new DriveSubsystem(hardwareMap);

        // 2. Set starting pose
        drive.setStartingPose(new Pose(x, y, heading));

        // 3. Build all paths
        PathChain myPath = drive.getFollower().pathBuilder()...build();

        // 4. Schedule the command sequence
        schedule(new SequentialCommandGroup(
                new FollowPathCommand(drive, myPath, true),
                new SomeOtherCommand(...)
        ));
    }
}
```

Alliance-color or game-piece selection must resolve in `initialize()` (select a pre-built path), never branch inside a running command.

---

## Contribution Rules

1. **Test on hardware before merging.** Simulation is unavailable for FTC. Untested changes go on a feature branch, not `main`.

2. **One logical change per commit.** "Added scoring command and fixed arm offset and updated path" is three commits.

3. **Re-run validation tests after any constants change.** Run `Line`, `Triangle`, and `Circle` from the `Tuning` OpMode after editing `FollowerConstants` or switching localizers.

4. **Delete dead code immediately.** Commented-out OpModes, unused commands, and orphaned constants accumulate fast and mislead future readers.

5. **Update this README** when a new subsystem or OpMode is added, or when the hardware configuration changes.
