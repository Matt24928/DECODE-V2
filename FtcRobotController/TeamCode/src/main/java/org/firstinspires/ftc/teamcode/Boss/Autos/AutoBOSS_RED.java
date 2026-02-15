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
public class AutoBOSS_RED extends CommandOpMode {

    private Follower follower;
    public OuttakeCommand outtake;
    public Intake intake;
    public static Pose LastPose;

    private final Pose startPose = new Pose(125.2, 120, Math.toRadians(43));
    private final Pose Score = new Pose(92.5, 95.2);


    public Pose Grab1 = new Pose(125.8, 77.8);
    public Pose Grab1ControlPoint = new Pose(89, 74.5);


    public Pose Grab2 = new Pose(125, 52.6);
    public Pose Grab2ControlPoint1 = new Pose(83.9, 72.4);
    public Pose Grab2ControlPoint2 = new Pose(96.7, 53.4);
    public Pose Score2Control = new Pose(111, 60);


    public Pose Grab3 = new Pose(126.3,34.5);
    public Pose Grab3ControlPoint = new Pose(84.1,23.1);

    public Pose Grab3ControlPoint2 = new Pose(103, 35.2);


    public Pose Empty = new Pose(131,73.8);
    public Pose EmptyControl = new Pose(117.3,71);


    public Pose Out = new Pose(107,91);


    public PathChain ScorePreload, GoTo1, Score1, GoTo2, Score2,GoTo3,GoToRamp,Score3, Exit;

    public SequentialCommandGroup Shoot1, Shoot2, Shootboth, ShootAll,ShootAll2;



    @Override
    public void initialize() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        outtake = new OuttakeCommand(hardwareMap);
        intake = new Intake(hardwareMap);
        outtake.SetAngle(0.28);
        outtake.targetRpm = 1100;

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
                new TimerCommand(200),
                new InstantCommand(outtake::Low1),
                new InstantCommand(outtake::Low2)
        );

        ShootAll = new SequentialCommandGroup(
                new TimerCommand(200),
                Shoot1,
                new InstantCommand(intake::eat),
                new TimerCommand(500),
                Shoot2,
                new TimerCommand(700),
                new InstantCommand(intake::zero),
                Shootboth,
                new InstantCommand(outtake::stop2)
        );


        ScorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, Score))
                .setConstantHeadingInterpolation(Math.toRadians(43))
                .setBrakingStrength(5)
                .addParametricCallback(0.05, ()-> outtake.ShootApr())
                .build();

        GoToRamp = follower.pathBuilder()
                .addPath(new BezierCurve(Grab1,EmptyControl, Empty))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addParametricCallback(0.7,()->intake.zero())
                .addPath(new BezierLine(Empty, Score))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(43))
                .setBrakingStrength(6)
                .addParametricCallback(0.1, ()->outtake.ShootApr())
                .build();

        GoTo1 = follower.pathBuilder()
                .addPath(new BezierCurve(Score, Grab1ControlPoint, Grab1))
                .setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(0),0.7)
                .addParametricCallback(0.01, ()->intake.eat())
                .setBrakingStrength(8)
                .setTValueConstraint(0.9)
                .setNoDeceleration()
                .build();

//        Score1 = follower.pathBuilder()
//                .addPath(new BezierLine(Empty, Score))
//                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(43))
//                .addParametricCallback(0.2, ()->intake.zero())
//                .addParametricCallback(0.05, ()->outtake.ShootApr())
//                .build();

        GoTo2 = follower.pathBuilder()
                .addPath(new BezierCurve(Score, Grab2ControlPoint1, Grab2ControlPoint2, Grab2))
                .setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(0), 0.5)
                .addParametricCallback(0.25, ()->intake.eat())
                .addPath(new BezierCurve(Grab2, Score2Control, Score))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(43))
                .setBrakingStrength(7)
                .addParametricCallback(0.5, ()->intake.zero())
                .addParametricCallback(0.1, ()->outtake.ShootApr())
                .build();

//        Score2 = follower.pathBuilder()
//                .addPath(new BezierCurve(Grab2, Score2Control, Score))
//                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(43))
//                .addParametricCallback(0.2, ()->intake.zero())
//                .addParametricCallback(0.05, ()->outtake.ShootApr())
//                .build();

        GoTo3 = follower.pathBuilder()
                .addPath(new BezierCurve(Score,Grab3ControlPoint,Grab3ControlPoint2,Grab3))
                .setLinearHeadingInterpolation(Math.toRadians(43),Math.toRadians(0),0.5)
                .addParametricCallback(0.25,()->intake.eat())
                .addPath(new BezierLine(Grab3,Score))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(43))
                .addParametricCallback(0.5,()->intake.zero())
                .setTranslationalConstraint(3.5)
                .setTValueConstraint(0.9)
                .setBrakingStrength(8)
                .addParametricCallback(0.5, ()->intake.zero())
                .addParametricCallback(0.1, ()->outtake.ShootApr())
                .build();

//        Score3 = follower.pathBuilder()
//                .addPath(new BezierLine(Grab3,Score))
//                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(43))
//                .addParametricCallback(0.2, ()->intake.zero())
//                .addParametricCallback(0.05, ()->outtake.ShootApr())
//                .build();

        Exit = follower.pathBuilder()
                .addPath(new BezierLine(Score, Out))
                .setConstantHeadingInterpolation(Math.toRadians(43))
                .setNoDeceleration()
                .build();

        schedule(new SequentialCommandGroup(
                new PedroFollowPath(follower,ScorePreload),
                ShootAll,
                new PedroFollowPath(follower,GoTo1),
                new PedroFollowPath(follower, GoToRamp),
                ShootAll,
                new PedroFollowPath(follower,GoTo2),
                ShootAll,
                new PedroFollowPath(follower,GoTo3),
                ShootAll,
                new PedroFollowPath(follower, Exit)
        ));
    }

    @Override
    public void run(){
        super.run();
        follower.update();
        telemetry.addData("path",follower.getCurrentPath());
        telemetry.addData("debug: ",follower.debug());
        telemetry.addData("current ticks",outtake.getCurrentTicks());
        telemetry.update();
        LastPose = follower.getPose();
    }

}