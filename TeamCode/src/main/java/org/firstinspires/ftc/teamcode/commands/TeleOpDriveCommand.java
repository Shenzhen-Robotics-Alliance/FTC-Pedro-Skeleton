package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * TeleOpDriveCommand
 *
 * FTCLib command that forwards gamepad input to DriveSubsystem during TeleOp.
 *
 * Supports two drive modes:
 *   - Robot-Centric: forward is always the robot's heading direction.
 *   - Field-Centric: forward is always a fixed field direction (requires odometry heading).
 */
public class TeleOpDriveCommand extends CommandBase {

    private final DriveSubsystem drive;

    // Input suppliers — evaluated each frame so values are always current
    private final DoubleSupplier forwardSup;      // left stick Y (up = positive)
    private final DoubleSupplier strafeSup;       // left stick X (right = positive)
    private final DoubleSupplier rotateSup;       // right stick X (clockwise = positive)
    private final BooleanSupplier robotCentricSup; // true = robot-centric, false = field-centric

    private static final double DEADBAND       = 0.05;
    private static final double SLOW_MULTIPLIER = 0.35;

    private double speedMultiplier = 1.0;

    /**
     * Constructor using raw DoubleSuppliers (works with any input source).
     *
     * @param drive           Drive subsystem
     * @param forwardSup      Forward/back axis input [-1, 1]
     * @param strafeSup       Strafe axis input [-1, 1]
     * @param rotateSup       Rotation axis input [-1, 1]
     * @param robotCentricSup Mode toggle: true = robot-centric
     */
    public TeleOpDriveCommand(
     DriveSubsystem drive,
     DoubleSupplier forwardSup,
     DoubleSupplier strafeSup,
     DoubleSupplier rotateSup,
     BooleanSupplier robotCentricSup) {
        this.drive = drive;
        this.forwardSup = forwardSup;
        this.strafeSup = strafeSup;
        this.rotateSup = rotateSup;
        this.robotCentricSup = robotCentricSup;
        addRequirements(drive);
    }

    /**
     * Convenience constructor using FTCLib GamepadEx.
     * Left stick Y/X -> forward/strafe; right stick X -> rotate.
     * Axes are negated to match standard FTC gamepad sign convention.
     *
     * @param drive           Drive subsystem
     * @param gamepad         FTCLib GamepadEx instance
     * @param robotCentricSup Mode toggle supplier
     */
    public TeleOpDriveCommand(DriveSubsystem drive,
                               GamepadEx gamepad,
                               BooleanSupplier robotCentricSup) {
        this(drive,
             () -> -gamepad.getLeftY(),
             () -> -gamepad.getLeftX(),
             () -> -gamepad.getRightX(),
             robotCentricSup);
    }

    @Override
    public void execute() {
        double forward = applyDeadband(forwardSup.getAsDouble()) * speedMultiplier;
        double strafe  = applyDeadband(strafeSup.getAsDouble())  * speedMultiplier;
        double rotate  = applyDeadband(rotateSup.getAsDouble())  * speedMultiplier;
        boolean robotCentric = robotCentricSup.getAsBoolean();

        drive.drive(forward, strafe, rotate, robotCentric);
    }

    /** Drive commands run indefinitely until interrupted. */
    @Override
    public boolean isFinished() {
        return false;
    }

    /** Stop motors when interrupted (e.g. an auto path command preempts this). */
    @Override
    public void end(boolean interrupted) {
        drive.drive(0, 0, 0, true);
    }

    public void enableSlowMode()  { speedMultiplier = SLOW_MULTIPLIER; }
    public void disableSlowMode() { speedMultiplier = 1.0; }

    /** Suppress stick drift: values inside the deadband are treated as zero. */
    private double applyDeadband(double value) {
        return Math.abs(value) < DEADBAND ? 0.0 : value;
    }
}
