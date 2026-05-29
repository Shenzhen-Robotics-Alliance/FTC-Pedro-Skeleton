package org.firstinspires.ftc.teamcode.opmodes.teleop;

// NOTE: The class is named TeleOp, which clashes with the annotation's simple name.
// We resolve this by using the fully-qualified annotation name instead of importing it.

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * Main driver-controlled OpMode.
 * Control layout — Gamepad 1 (Driver):
 *   Left  stick  Y / X    Forward / Strafe
 *   Right stick  X        Rotate
 *   Left  bumper (hold)   Robot-centric mode override
 *   Right bumper (hold)   Slow mode  (35 % speed)
 *   Back  button          Reset field-centric heading
 * Default drive mode is field-centric (forward = away from the driver wall).
 * Hold LEFT_BUMPER at any time to switch to robot-centric.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "OpModes")
public class TeleOp extends CommandOpMode {

    @Override
    public void initialize() {

        // ── Hardware ──────────────────────────────────────────────────────────
        DriveSubsystem drive  = new DriveSubsystem(hardwareMap);
        GamepadEx      driver = new GamepadEx(gamepad1);

        drive.startTeleOpDrive();

        // ── Drive command ─────────────────────────────────────────────────────
        // Field-centric by default; robot-centric while LEFT_BUMPER is held.
        TeleOpDriveCommand driveCommand = new TeleOpDriveCommand(
                drive,
                driver,
                () -> driver.getButton(GamepadKeys.Button.LEFT_BUMPER)
        );
        drive.setDefaultCommand(driveCommand);

        // ── Button bindings ───────────────────────────────────────────────────

        // RIGHT_BUMPER (hold) → 35 % speed cap
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(driveCommand::enableSlowMode)
                .whenReleased(driveCommand::disableSlowMode);

        // BACK → re-zero field-centric heading (keeps X/Y position intact)
        driver.getGamepadButton(GamepadKeys.Button.BACK)
                .whenPressed(drive::resetHeading);

        // ── Telemetry ─────────────────────────────────────────────────────────
        // Anonymous subsystem: periodic() is called every scheduler tick,
        // which is the FTCLib-idiomatic way to push telemetry each loop.
        register(new SubsystemBase() {
            @Override
            public void periodic() {
                telemetry.addLine("─── Drive ───────────────────────────");
                telemetry.addData("  Mode",
                        driver.getButton(GamepadKeys.Button.LEFT_BUMPER)
                                ? "Robot-Centric" : "Field-Centric");
                telemetry.addData("  Speed",
                        driver.getButton(GamepadKeys.Button.RIGHT_BUMPER)
                                ? "Slow (35%)" : "Full (100%)");
                telemetry.addLine("─── Pose ────────────────────────────");
                telemetry.addData("  X   (in)",  String.format("%.2f", drive.getPose().getX()));
                telemetry.addData("  Y   (in)",  String.format("%.2f", drive.getPose().getY()));
                telemetry.addData("  Hdg (deg)", String.format("%.1f", Math.toDegrees(drive.getPose().getHeading())));
                telemetry.update();
            }
        });
    }
}
