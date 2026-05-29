package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@Autonomous(name = "AutoSample", group = "OpModes")
public class AutoSample extends CommandOpMode {

    // Starting pose — adjust to match your alliance/field position
    private static final Pose START_POSE = new Pose(0, 0, 0);

    @Override
    public void initialize() {
        DriveSubsystem drive = new DriveSubsystem(hardwareMap);
        drive.setStartingPose(START_POSE);

        // Define waypoint poses
        Pose startPose  = new Pose(0,  0,  0);
        Pose midPose    = new Pose(48, 0,  0);
        Pose cp1        = new Pose(48, 24, 0);  // BezierCurve control point
        Pose endPose    = new Pose(24, 24, 0);

        // Example path: drive forward 48 in, then curve left 24 in
        PathChain autoPath = drive.getFollower().pathBuilder()
                .addPath(new BezierLine(startPose, midPose))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierCurve(midPose, cp1, endPose))
                .setLinearHeadingInterpolation(0, Math.PI / 2)
                .build();

        schedule(new SequentialCommandGroup(
                new FollowPathCommand(drive, autoPath, true)
        ));
    }
}
