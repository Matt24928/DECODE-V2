package org.firstinspires.ftc.teamcode.Pluto.Autos;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Commands.PedroFollowPath;
import org.firstinspires.ftc.teamcode.Commands.TimerCommand;
import org.firstinspires.ftc.teamcode.Pluto.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.Pluto.SubSystems.Limelight;
import org.firstinspires.ftc.teamcode.Pluto.SubSystems.Outtake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(group = "Main close")
public class AUTO_CLOSE_RED_LOADING_LL extends CommandOpMode {

    private Follower follower;
    private Outtake outtake;
    private Intake intake;

    public Limelight limelight;

    private ElapsedTime Timer_Until_Chase, Time_To_Shoot;
    public boolean auto = false, idk = true, finished = false, reset = true, time_shoot = true, tre_sa_iasa = true;


    private Pose startingPose = new Pose(129.6, 116.3, Math.toRadians(180));
    private Pose Shoot = new Pose(82.8, 83.6, Math.toRadians(-136));


    private Pose Grab1 = new Pose(118.6, 83.5);


    private Pose Grab2 = new Pose(123.6, 59);
    private Pose Grab2Control = new Pose(88.2, 56);


    private Pose Grab3 = new Pose(125, 33.8);
    private Pose Grab3Control = new Pose(90.5, 27.7);

    private Pose Go_To_Tunnel = new Pose(134,57.4);
    private Pose Go_To_Loading_Zone1 = new Pose(137, 13);
    private Pose Go_To_Loading_Zone2 = new Pose(120.1, 16.3);


    private Pose Open_Gate = new Pose(133.1, 73.5);
    private Pose Open_Gate_Control = new Pose(117.6, 63.3);


    private Pose poro = new Pose(86, 56.1);

//    private Pose Leave_Auto = new Pose(80.6, 90.8);

    public static Pose lastpose;

    private PathChain scorePreload, Get1, Get2, Score1, Score2, Get_and_score3, Cycle, Cycle2;

    public Pose Get_Artifact;
    private PathChain getArtefact, Start_Scan, Leave_Auto, Start_Scan_cu_LL;

//    private Pose Leave = new Pose(63.4, 90.8);

    private Pose Stalk = new Pose(124.1, 70.7);

    private Pose iesi = new Pose(89.2, 56.1);


    @Override
    public void initialize(){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        outtake = new Outtake(hardwareMap);
        intake = new Intake(hardwareMap);
        outtake.VEL_APR = 1100;
        limelight = new Limelight(hardwareMap);
        limelight.limelight.pipelineSwitch(0);
        Timer_Until_Chase = new ElapsedTime();
        Time_To_Shoot = new ElapsedTime();

        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startingPose, Shoot))
                .setLinearHeadingInterpolation(Math.toRadians(180), Shoot.getHeading())
                .addParametricCallback(0.05, ()-> outtake.Shoot_Apr())
                .addParametricCallback(0.1, ()-> outtake.Up())
                .setBrakingStart(0.05)
                .setVelocityConstraint(0.5)
                .setTValueConstraint(0.7)
                .setTimeoutConstraint(50)
                .setHeadingConstraint(0.05)
                .setTranslationalConstraint(0.5)
                .build();

        Get1 = follower.pathBuilder()
                .addPath(new BezierLine(Shoot, Grab1))
                .setLinearHeadingInterpolation(Shoot.getHeading(), Math.toRadians(0), 0.1)
                .addParametricCallback(0.05, ()-> outtake.Down())
                .addPath(new BezierLine(Grab1, Shoot))
                .setLinearHeadingInterpolation(Math.toRadians(0), Shoot.getHeading())
                .addParametricCallback(0.8, ()-> intake.Stop())
                .addParametricCallback(0.95, ()-> outtake.Up())
                .setBrakingStart(0.3)
                .setVelocityConstraint(0.15)
                .setTValueConstraint(0.7)
                .setTimeoutConstraint(50)
                .setHeadingConstraint(0.05)
                .setTranslationalConstraint(0.5)
                .build();

//        Score1 =follower.pathBuilder()
//                .addPath(new BezierLine(Grab1, Shoot))
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(316))
//                .addParametricCallback(0.8, ()-> intake.Stop())
//                .addParametricCallback(0.95, ()-> outtake.Up())
//                .setBrakingStrength(4)
//                .setBrakingStart(0.3)
//                .setVelocityConstraint(0.15)
//                .setTValueConstraint(0.7)
//                .setTimeoutConstraint(50)
//                .setHeadingConstraint(0.05)
//                .setTranslationalConstraint(0.5)
//                .build();

        Get2 = follower.pathBuilder()
                .addPath(new BezierCurve(Shoot, Grab2Control, Grab2))
                .setLinearHeadingInterpolation(Shoot.getHeading(),Math.toRadians(0), 0.1)
                .addParametricCallback(0.05, ()-> outtake.Down())
                .addPath(new BezierCurve(Grab2, Open_Gate_Control, Open_Gate))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-84), 0.1)
                .addPath(new BezierLine(Open_Gate, Shoot))
                .setLinearHeadingInterpolation(Math.toRadians(-96), Shoot.getHeading())
                .addParametricCallback(0.8, ()-> intake.Stop())
                .addParametricCallback(0.95, ()-> outtake.Up())
                .setBrakingStart(0.3)
                .setVelocityConstraint(0.5)
                .setTValueConstraint(0.7)
                .setTimeoutConstraint(50)
                .setHeadingConstraint(0.05)
                .setTranslationalConstraint(0.5)
                .build();

//        Score2 = follower.pathBuilder()
//                .addPath(new BezierLine(Open_Gate, Shoot))
//                .setLinearHeadingInterpolation(Math.toRadians(-96), Math.toRadians(-138))
//                .addParametricCallback(0.8, ()-> intake.Stop())
//                .addParametricCallback(0.95, ()-> outtake.Up())
//                .setBrakingStart(0.3)
//                .setVelocityConstraint(0.5)
//                .setTValueConstraint(0.7)
//                .setTimeoutConstraint(50)
//                .setHeadingConstraint(0.05)
//                .setTranslationalConstraint(0.5)
//                .build();

        Get_and_score3 = follower.pathBuilder()
                .addPath(new BezierCurve(Shoot, Grab3Control, Grab3))
                .setLinearHeadingInterpolation(Shoot.getHeading(), Math.toRadians(0), 0.1)
                .addParametricCallback(0.05, ()-> outtake.Down())
                .addPath(new BezierLine(Grab3, Shoot))
                .setLinearHeadingInterpolation(Math.toRadians(0), Shoot.getHeading())
                .addParametricCallback(0.2, ()-> intake.Stop())
                .addParametricCallback(0.95, ()-> outtake.Up())
                .setBrakingStart(0.3)
                .setVelocityConstraint(0.5)
                .setTValueConstraint(0.7)
                .setTimeoutConstraint(50)
                .setHeadingConstraint(0.05)
                .setTranslationalConstraint(0.5)
                .build();

        Start_Scan = follower.pathBuilder()
                .addPath(new BezierLine(Shoot, Stalk))
                .setLinearHeadingInterpolation(Shoot.getHeading(), Math.toRadians(270), 0.1)
                .build();



        schedule(new SequentialCommandGroup(
                new PedroFollowPath(follower, scorePreload),
                new TimerCommand(100),
                new InstantCommand(intake::Eat),
                new TimerCommand(300),
                new PedroFollowPath(follower, Get1),
                new TimerCommand(100),
                new InstantCommand(intake::Eat),
                new TimerCommand(300),
                new PedroFollowPath(follower, Get2),
                new TimerCommand(100),
                new InstantCommand(intake::Eat),
                new TimerCommand(300),
                new PedroFollowPath(follower, Get_and_score3),
                new TimerCommand(100),
                new InstantCommand(intake::Eat),
                new TimerCommand(300),
                new PedroFollowPath(follower, Start_Scan)
        ));

    }

    void BuildPath() {
        getArtefact = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), Get_Artifact))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), Math.toRadians(0), 0.1)
                .addParametricCallback(0.2, () -> {
                    intake.Eat();
                })
                .addParametricCallback(0.05, ()-> outtake.Down())
//                .addPath(new BezierLine(Get_Artifact, get))
//                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(new BezierLine(Get_Artifact, Shoot))
                .setLinearHeadingInterpolation(Math.toRadians(0), Shoot.getHeading())
                .addParametricCallback(0.2, () -> intake.Stop())
                .addParametricCallback(0.8, () -> outtake.Up())
                .addParametricCallback(0.9,()->{finished = true;})
                .setTValueConstraint(0.9)
                .build();

        Start_Scan_cu_LL = follower.pathBuilder()
                .addPath(new BezierLine(Shoot, Stalk))
                .setLinearHeadingInterpolation(Shoot.getHeading(), Math.toRadians(270), 0.1)
                .addParametricCallback(0.85, ()->limelight.limelight.start())
                .addParametricCallback(0.9, ()->{auto = false;})
                .build();

        Leave_Auto = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), iesi))
                .setConstantHeadingInterpolation(Math.toRadians(follower.getHeading()))
                .build();
    }

    @Override
    public void run(){
        super.run();
        follower.update();
        limelight.periodic();
        limelight.getResults();

        if(reset)  {
            Timer_Until_Chase.reset();
            reset = false;
        }

        if(Timer_Until_Chase.seconds() > 17 && Timer_Until_Chase.seconds() < 28.0) {
            if (Timer_Until_Chase.seconds() > 17 && Timer_Until_Chase.seconds() < 28.0 && limelight.hasTarget() && !auto) {
                auto = true;
                limelight.limelight.stop();
//            limelight.Rich_Artefact(follower, xField, yField);
                double robotX = follower.getPose().getX(); // cum merge el in fata]
                double robotY = follower.getPose().getY(); // cum merge el in laterala
                // double headingRad = follower.getPose().getHeading();

                double xField, yField;

                if (limelight.DY < 0) {
                    yField = robotY + limelight.DY + limelight.CamOffsetFromCenter;}
                else{
                    yField = robotY + limelight.DY - limelight.CamOffsetFromCenter;
                }

                xField = robotX;
                Get_Artifact = new Pose(follower.getPose().getX() + limelight.DX - 17, yField);
                BuildPath();
                follower.followPath(getArtefact);
            }
            if(finished) {
                if (time_shoot) {
                    Time_To_Shoot.reset();
                    time_shoot = false;
                }
                if (Time_To_Shoot.seconds() > 1.5 && auto) {
                    finished = false;
                    time_shoot = true;
                    BuildPath();
                    follower.followPath(Start_Scan_cu_LL);
                } else if (Time_To_Shoot.seconds() > 0.5 && Time_To_Shoot.seconds() < 1.5 && Timer_Until_Chase.seconds() > 13.0) {
                    intake.Eat();
                }
            }
        }

        if(Timer_Until_Chase.seconds() > 28 && tre_sa_iasa) {
            BuildPath();
            follower.followPath(Leave_Auto);
            tre_sa_iasa = false;
        }
        lastpose = follower.getPose();
        telemetry.addData("path", follower.getCurrentPath());
        telemetry.addData("Secunde: ", Timer_Until_Chase.seconds());
        telemetry.addData("Time until shoot: ", Time_To_Shoot.seconds());
        telemetry.addData("Finished: ", finished);
        telemetry.addData("idk: ", idk);
        telemetry.addData("auto: ", auto);
        telemetry.addData("Has target", limelight.hasTarget());
        telemetry.addData("DX", limelight.DX);
        telemetry.addData("DY", limelight.DY);
        telemetry.addData("path",follower.getCurrentPath());
        telemetry.addData("is on: ", limelight.limelight.isRunning());
        telemetry.update();

        lastpose = follower.getPose();
    }}

