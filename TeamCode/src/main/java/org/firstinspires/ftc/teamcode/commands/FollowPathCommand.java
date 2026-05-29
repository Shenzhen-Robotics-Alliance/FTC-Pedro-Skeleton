package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

/**
 * FTCLib command that follows a PedroPathing Path or PathChain.
 * Finishes when the follower reports it is no longer busy.
 */
public class FollowPathCommand extends CommandBase {

    private final DriveSubsystem drive;
    private final Path path;
    private final PathChain pathChain;
    private final boolean holdEnd;

    public FollowPathCommand(DriveSubsystem drive, Path path, boolean holdEnd) {
        this.drive = drive;
        this.path = path;
        this.pathChain = null;
        this.holdEnd = holdEnd;
        addRequirements(drive);
    }

    public FollowPathCommand(DriveSubsystem drive, PathChain pathChain, boolean holdEnd) {
        this.drive = drive;
        this.path = null;
        this.pathChain = pathChain;
        this.holdEnd = holdEnd;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        if (pathChain != null) {
            drive.followPath(pathChain, holdEnd);
        } else {
            drive.followPath(path, holdEnd);
        }
    }

    @Override
    public boolean isFinished() {
        return !drive.isBusy();
    }
}
