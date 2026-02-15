package org.firstinspires.ftc.teamcode.Boss.Autos;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Commands.PedroFollowPath;

import org.firstinspires.ftc.teamcode.Commands.TimerCommand;
import org.firstinspires.ftc.teamcode.Boss.Autos.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Boss.Autos.Subsystems.OuttakeCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous
@Disabled
public class AutoBOSS_FAR_RED extends CommandOpMode {

    private Follower follower;
    public OuttakeCommand outtake;
    public Intake intake;

    private final Pose startPose = new Pose(88, 8, Math.toRadians(90));
    private final Pose Score = new Pose(85, 17);
    private final Pose Score2 = new Pose(85,17.3);


    public Pose Grab1 = new Pose(131.6, 18.4);
    public Pose Grab1ControlPoint = new Pose(92.3, 23.7);
    public Pose GoGet1 = new Pose(135.6, 12.4);
    public Pose GetSpike1 = new Pose(129.5,35.6);
    public Pose GetSpike1Control = new Pose(83,37);


    public Pose Exit_Launching_Zone = new Pose(83, 37);


    public PathChain ScorePreload, GoTo1, Score1, Exit, Get1, Spike,    Score_2;

    public SequentialCommandGroup Shoot1, Shoot2, Shootboth, ShootAll;
    public static Pose LastPose;



    @Override
    public void initialize() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        outtake = new OuttakeCommand(hardwareMap);
        intake = new Intake(hardwareMap);
        outtake.targetRpm = 1350;
        outtake.ANG_departe = 0.25;
        outtake.AngleDep();

        Shoot1 = new SequentialCommandGroup(
                new InstantCommand(outtake::Jump1),
                new TimerCommand(400),
                new InstantCommand(outtake::Low1)
        );
        Shoot2 = new SequentialCommandGroup(
                new InstantCommand(outtake::Jump2),
                new TimerCommand(400),
                new InstantCommand(outtake::Low2)
        );

        Shootboth = new SequentialCommandGroup(
                new InstantCommand(outtake::Jump1),
                new InstantCommand(outtake::Jump2),
                new TimerCommand(300),
                new InstantCommand(outtake::Low1),
                new InstantCommand(outtake::Low2)
        );

        ShootAll = new SequentialCommandGroup(
                new TimerCommand(1500),
                Shoot1,
                new InstantCommand(intake::eat),
                new TimerCommand(900),
                Shoot2,
                new TimerCommand(900),
                new InstantCommand(intake::zero),
                Shootboth,
                new InstantCommand(outtake::stop2)
        );


        ScorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, Score))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(67))
                .setBrakingStrength(3)
                .addParametricCallback(0.05, ()-> outtake.ShootApr())
                .build();

        GoTo1 = follower.pathBuilder()
                .addPath(new BezierCurve(Score, Grab1ControlPoint, Grab1))
                .setLinearHeadingInterpolation(Math.toRadians(67), Math.toRadians(310))
                .addParametricCallback(0.01, ()->intake.eat())
                .build();

        Get1 = follower.pathBuilder()
                .addPath(new BezierLine(Grab1, GoGet1))
                .setLinearHeadingInterpolation(Math.toRadians(310), Math.toRadians(270))
                .build();

        Score1 = follower.pathBuilder()
                .addPath(new BezierLine(GoGet1, Score))
                .setConstantHeadingInterpolation(Math.toRadians(270))
                .addParametricCallback(0.2, ()->intake.zero())
                .addParametricCallback(0.05, ()->outtake.ShootApr())
                .addPath(new BezierLine(Score, Score2))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(67))
                .build();

        Score_2 = follower.pathBuilder()
                .addPath(new BezierLine(GetSpike1, Score))
                .addParametricCallback(0.1, ()-> outtake.Shoot())
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(67))
                .build();

        Exit = follower.pathBuilder()
                .addPath(new BezierLine(Score2, Exit_Launching_Zone))
                .setConstantHeadingInterpolation(Math.toRadians(67))
                .build();

        Spike = follower.pathBuilder()
                .addPath(new BezierCurve(Score, GetSpike1Control, GetSpike1))
                .setLinearHeadingInterpolation(Math.toRadians(67), Math.toRadians(0))
                .addParametricCallback(0.01, () -> intake.eat())
                .build();

        schedule(new SequentialCommandGroup(
                new PedroFollowPath(follower,ScorePreload),
                ShootAll,
                new PedroFollowPath(follower, Spike),
                new PedroFollowPath(follower, Score_2),
                ShootAll,
                new PedroFollowPath(follower, GoTo1),
                new PedroFollowPath(follower,Get1),
                new TimerCommand(1000),
                new PedroFollowPath(follower, Score1),
                ShootAll,
                new PedroFollowPath(follower, Exit)

        ));
    }

    @Override
    public void run(){
        super.run();
        follower.update();
        telemetry.addData("path",follower.getCurrentPath());
        telemetry.update();
        LastPose = follower.getPose();
    }

}