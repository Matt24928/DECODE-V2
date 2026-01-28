package org.firstinspires.ftc.teamcode.Autos;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import android.graphics.Point;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.gamepad.PanelsGamepad;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.GoodLimelight;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Configurable
@Autonomous
public class Auto_RED extends OpMode {
    GoodLimelight vision;
    private Outtake outtake;
    private Follower follower;
    private Intake intake;
    private final Pose startPose = new Pose(88,8,Math.toRadians(90));
    private static PathChain ScorePreLoad,Path2,Path3,Path4,Path5;
    private static int pathState;
    public static Outtake.Patterns CurrentPattern;
    private Timer pathTimer, actionTimer, opmodeTimer;
    public static boolean GoOn = false;
    public enum Shootceva{
        SHOOTING,
        IDLE
    }
    private Shootceva shootState = Shootceva.IDLE;



    public void BuildPaths(){

        ScorePreLoad = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(88.000, 8.000),

                                new Pose(85.000, 17.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(65))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(59.000, 17.000),
                                new Pose(62.500, 39.7)

                        )
                ).setLinearHeadingInterpolation(Math.toRadians(65), Math.toRadians(90))
                .build();



    }
    @Override
    public void init() {
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        BuildPaths();
    }
    @Override
    public void init_loop(){
    }
    @Override
    public void start(){
        opmodeTimer.resetTimer();
        setPathState(0);
    }
    @Override
    public void loop() {
        outtake.Update();
        outtake.OuttakeData(telemetry);
        intake.Update();
        follower.update();
        autonomousPathUpdate();
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
    public void setPathState(int pState) {
        pathState = pState;
        actionTimer.resetTimer();
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(ScorePreLoad);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    outtake.AngleDep();
                    if (shootState == Shootceva.IDLE) {
                        outtake.ShootOverride();   // o singură dată
                        shootState = Shootceva.SHOOTING;
                    }
                    if (shootState == Shootceva.SHOOTING && outtake.overrideShootState == Outtake.OverrideShootState.IDLE) {
                        shootState = Shootceva.IDLE;
                        // follower.followPath(Path2);
                        setPathState(2);
                    }
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    if(pathTimer.getElapsedTimeSeconds() > 25){
                        // intake.zero();
                        follower.followPath(Path3,true);
                        setPathState(-1);}
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    if (shootState == Shootceva.IDLE) {
                        outtake.ShootOverride();   // o singură dată
                        shootState = Shootceva.SHOOTING;
                    }
                    if (shootState == Shootceva.SHOOTING && outtake.overrideShootState == Outtake.OverrideShootState.IDLE) {
                        shootState = Shootceva.IDLE;
                        setPathState(-1);
                    }

                }
                break;


        }
    }
}
