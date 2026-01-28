package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;

public class PedroFollowPath extends CommandBase {

    private final Follower follower;
    private final PathChain path;

    public PedroFollowPath(Follower follower, PathChain path) {
        this.follower = follower;
        this.path = path;
    }

    @Override
    public void initialize() {
        follower.followPath(path);
    }

    @Override
    public boolean isFinished() {
        return !follower.isBusy();
    }
}
