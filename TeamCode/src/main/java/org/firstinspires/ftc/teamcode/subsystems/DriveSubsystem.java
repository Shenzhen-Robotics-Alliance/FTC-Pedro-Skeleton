package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * DriveSubsystem wraps the PedroPathing Follower as an FTCLib subsystem.
 *
 * Motor naming expected in the hardware config:
 *   "leftFront", "rightFront", "leftBack", "rightBack"
 *
 * Localizer and follower constants come from {@link Constants}.
 */
public class DriveSubsystem extends SubsystemBase {

    private final Follower follower;

    /**
     * @param hardwareMap OpMode hardware map used to initialize the follower.
     */
    public DriveSubsystem(HardwareMap hardwareMap) {
        this.follower = Constants.createFollower(hardwareMap);
    }

    /** Set the robot's starting pose before autonomous begins. */
    public void setStartingPose(Pose pose) {
        follower.setStartingPose(pose);
    }

    /** Switch the follower into TeleOp drive mode. */
    public void startTeleOpDrive() {
        follower.startTeleOpDrive();
    }

    /**
     * Drive the robot with cartesian vectors.
     *
     * @param forward     Forward/back power [-1, 1]
     * @param strafe      Left/right power   [-1, 1]
     * @param rotate      Rotation power     [-1, 1]
     * @param robotCentric true = robot-centric, false = field-centric
     */
    public void drive(double forward, double strafe, double rotate, boolean robotCentric) {
        follower.setTeleOpDrive(forward, strafe, rotate, robotCentric);
    }

    /** Follow a single Path, optionally holding position at the end. */
    public void followPath(Path path, boolean holdEnd) {
        follower.followPath(path, holdEnd);
    }

    /** Follow a PathChain, optionally holding position at the end. */
    public void followPath(PathChain pathChain, boolean holdEnd) {
        follower.followPath(pathChain, holdEnd);
    }

    /** @return true while the follower is actively tracking a path. */
    public boolean isBusy() {
        return follower.isBusy();
    }

    /** @return Current estimated robot pose from the localizer. */
    public Pose getPose() {
        return follower.getPose();
    }

    /**
     * Re-zero the field-centric heading reference while preserving X/Y position.
     * Call this when the odometry heading has drifted or after a hard collision.
     * Bind to a driver button (e.g. BACK) in the OpMode.
     */
    public void resetHeading() {
        Pose current = follower.getPose();
        follower.setStartingPose(new Pose(current.getX(), current.getY(), 0));
    }

    /** Expose the raw Follower for path building: {@code drive.getFollower().pathBuilder()}. */
    public Follower getFollower() {
        return follower;
    }

    /**
     * Called every loop by the FTCLib CommandScheduler.
     * Keeps the localizer and PID controllers updated.
     */
    @Override
    public void periodic() {
        follower.update();
    }
}
