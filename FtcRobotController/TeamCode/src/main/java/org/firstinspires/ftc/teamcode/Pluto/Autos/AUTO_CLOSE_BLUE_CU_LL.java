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
public class AUTO_CLOSE_BLUE_CU_LL extends CommandOpMode {

    private Follower follower;
    private Outtake outtake;
    private Intake intake;

    private Limelight limelight;

    private ElapsedTime Timer_Until_Chase, Time_To_Shoot;
    public boolean auto = false, idk = true, finished = false, reset = true, time_shoot = true, tre_sa_iasa = true;

    private Pose startingPose = new Pose(14.4, 116.3, Math.toRadians(0));
    private Pose Shoot = new Pose(61.2, 83.6);
    public static Pose lastpose;


    private Pose Grab1 = new Pose(26.4, 83.5);


    private Pose Grab2 = new Pose(20.4, 59);
    private Pose Grab2Control = new Pose(55.8, 56);


    private Pose Grab3 = new Pose(19, 33.8);
    private Pose Grab3Control = new Pose(53.5, 27.7);

    private Pose Go_To_Tunnel = new Pose(10,57.4);
    private Pose Go_To_Loading_Zone = new Pose(6, 8.4);


    private Pose Open_Gate = new Pose(7, 73.5);
    private Pose Open_Gate_Control = new Pose(26.4, 63.3);

    private Pose Leave = new Pose(63.4, 90.8);

    private Pose Stalk = new Pose(54.8, 56.1);

    private Pose iesi = new Pose(54.8, 56.1);


    private PathChain scorePreload, Get1, Get2, Score1, Score2, Get_and_score3, Start_Scan_cu_LL,   Cycle, Cycle2, Leave_Auto;
    public Pose Get_Artifact;
    private PathChain getArtefact, Start_Scan;

    @Override
    public void initialize(){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        outtake = new Outtake(hardwareMap);
        intake = new Intake(hardwareMap);
        limelight = new Limelight(hardwareMap);
        limelight.limelight.pipelineSwitch(0);
        Timer_Until_Chase = new ElapsedTime();
        Time_To_Shoot = new ElapsedTime();

        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startingPose, Shoot))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(318))
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
                .setLinearHeadingInterpolation(Math.toRadians(318), Math.toRadians(180), 0.1)
                .addParametricCallback(0.05, ()-> outtake.Down())
                .addPath(new BezierLine(Grab1, Shoot))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(318))
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
                .setLinearHeadingInterpolation(Math.toRadians(318),Math.toRadians(180), 0.1)
                .addParametricCallback(0.05, ()-> outtake.Down())
                .addPath(new BezierCurve(Grab2, Open_Gate_Control, Open_Gate))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(276), 0.1)
                .addParametricCallback(0.8, ()-> intake.Stop())
                .build();

        Score2 = follower.pathBuilder()
                .addPath(new BezierLine(Open_Gate, Shoot))
                .setLinearHeadingInterpolation(Math.toRadians(276), Math.toRadians(318))
                .addParametricCallback(0.8, ()-> intake.Stop())
                .addParametricCallback(0.95, ()-> outtake.Up())
                .setBrakingStart(0.3)
                .setVelocityConstraint(0.5)
                .setTValueConstraint(0.7)
                .setTimeoutConstraint(50)
                .setHeadingConstraint(0.05)
                .setTranslationalConstraint(0.5)
                .build();

        Leave_Auto = follower.pathBuilder()
                .addPath(new BezierCurve(Shoot, Grab3Control, Grab3))
                .setLinearHeadingInterpolation(Math.toRadians(318), Math.toRadians(180), 0.1)
                .addParametricCallback(0.05, ()-> outtake.Down())
                .addPath(new BezierLine(Grab3, Leave))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(318))
                .addParametricCallback(0.2, ()-> intake.Stop())
                .addParametricCallback(0.95, ()-> outtake.Up())
                .setBrakingStart(0.3)
                .setVelocityConstraint(0.5)
                .setTValueConstraint(0.7)
                .setTimeoutConstraint(50)
                .setHeadingConstraint(0.05)
                .setTranslationalConstraint(0.5)
                .build();

//        Cycle = follower.pathBuilder()
//                .addPath(new BezierLine(Shoot, Go_To_Tunnel))
//                .setLinearHeadingInterpolation(Math.toRadians(318), Math.toRadians(270), 0.1)
//                .addParametricCallback(0.05, ()-> outtake.Down())
//                .addPath(new BezierLine(Go_To_Tunnel, Go_To_Loading_Zone))
//                .setConstantHeadingInterpolation(Math.toRadians(270))
//                .addPath(new BezierLine(Go_To_Loading_Zone, Shoot))
//                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(318),0.3)
//                .addParametricCallback(0.2, ()-> intake.Stop())
//                .addParametricCallback(0.95, ()-> outtake.Up())
//                .setBrakingStart(0.3)
//                .setVelocityConstraint(0.5)
//                .setTValueConstraint(0.7)
//                .setTimeoutConstraint(50)
//                .setHeadingConstraint(0.05)
//                .setTranslationalConstraint(0.5)
//                .build();
//
//        Cycle2 = follower.pathBuilder()
//                .addPath(new BezierLine(Shoot, Go_To_Tunnel))
//                .setLinearHeadingInterpolation(Math.toRadians(318), Math.toRadians(270), 0.1)
//                .addParametricCallback(0.05, ()-> outtake.Down())
//                .addPath(new BezierLine(Go_To_Tunnel, Go_To_Loading_Zone))
//                .setConstantHeadingInterpolation(Math.toRadians(270))
//                .addPath(new BezierLine(Go_To_Loading_Zone, Leave))
//                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(318),0.3)
//                .addParametricCallback(0.2, ()-> intake.Stop())
//                .addParametricCallback(0.95, ()-> outtake.Up())
//                .setBrakingStart(0.3)
//                .setVelocityConstraint(0.5)
//                .setTValueConstraint(0.7)
//                .setTimeoutConstraint(50)
//                .setHeadingConstraint(0.05)
//                .setTranslationalConstraint(0.5)
//                .build();

        Start_Scan = follower.pathBuilder()
                .addPath(new BezierLine(Shoot, Stalk))
                .setLinearHeadingInterpolation(Math.toRadians(318), Math.toRadians(0), 0.1)
                .build();


        schedule(new SequentialCommandGroup(
                new PedroFollowPath(follower, scorePreload),
                new TimerCommand(300),
                new InstantCommand(intake::Eat),
                new TimerCommand(200),
                new PedroFollowPath(follower, Get1),
                new TimerCommand(300),
                new InstantCommand(intake::Eat),
                new TimerCommand(200),
                new PedroFollowPath(follower, Get2),
                new PedroFollowPath(follower, Score2),
                new TimerCommand(300),
                new InstantCommand(intake::Eat),
                new TimerCommand(200),
                new PedroFollowPath(follower, Start_Scan)
        ));

    }

    void BuildPath() {
        getArtefact = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), Get_Artifact))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), Math.toRadians(180), 0.1)
                .addParametricCallback(0.2, () -> {
                    intake.Eat();
                })
                .addParametricCallback(0.05, ()-> outtake.Down())
//                .addPath(new BezierLine(Get_Artifact, get))
//                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(new BezierLine(Get_Artifact, Shoot))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(318))
                .addParametricCallback(0.2, () -> intake.Stop())
                .addParametricCallback(0.8, () -> outtake.Up())
                .addParametricCallback(0.9,()->{finished = true;})
                .setTValueConstraint(0.9)
                .build();

        Start_Scan_cu_LL = follower.pathBuilder()
                .addPath(new BezierLine(Shoot, Stalk))
                .setLinearHeadingInterpolation(Math.toRadians(318), Math.toRadians(0), 0.1)
                .addParametricCallback(0.85, ()->limelight.limelight.start())
                .addParametricCallback(0.9, ()->{auto = false;})
                .build();

        Leave_Auto = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), iesi))
                .setConstantHeadingInterpolation(Math.toRadians(follower.getHeading()))
                .build();
    }


    @Override
    public void run() {
        super.run();
        follower.update();
        limelight.periodic();
        limelight.getResults();

        if(reset)  {
            Timer_Until_Chase.reset();
            reset = false;
        }

        if(Timer_Until_Chase.seconds() > 15 && Timer_Until_Chase.seconds() < 28.0) {
            if (Timer_Until_Chase.seconds() > 15 && Timer_Until_Chase.seconds() < 28.0 && limelight.hasTarget() && !auto) {
                auto = true;
                limelight.limelight.stop();
//            limelight.Rich_Artefact(follower, xField, yField);
                double robotX = follower.getPose().getX(); // cum merge el in fata]
                double robotY = follower.getPose().getY(); // cum merge el in laterala
                // double headingRad = follower.getPose().getHeading();

                double xField, yField;

                if (limelight.DY < 0) {
                    yField = robotY + Math.abs(limelight.DY) + limelight.CamOffsetFromCenter;
                }
                else{
                    yField = robotY - limelight.DY + limelight.CamOffsetFromCenter;
                }

                xField = robotX;
                Get_Artifact = new Pose(follower.getPose().getX() - limelight.DX + 17, yField);
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
        telemetry.addData("is on ", limelight.limelight.isRunning());
        telemetry.update();

    }
}
