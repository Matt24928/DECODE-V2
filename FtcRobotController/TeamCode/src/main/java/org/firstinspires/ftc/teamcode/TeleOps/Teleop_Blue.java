package org.firstinspires.ftc.teamcode.TeleOps;


import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.GoodLimelight;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Subsystems.Sorter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
@Configurable
public class Teleop_Blue extends OpMode {
    public static Outtake.Patterns CurrentPattern;
    PanelsTelemetry debug;
    GoodLimelight vision;
    // Telemetry telemetry;
    public static Pose startingPose = new Pose(59,39.7,Math.toRadians(90));
    public Sorter sorter;
    public Outtake outtake;
    public Intake intake;
    public Limelight limelight;
    private boolean automatedDrive = false;

    // Slow mode (opțional)
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    private Gamepad currentGamepad1 = new Gamepad();
    private Gamepad previousGamepad1 = new Gamepad();

    private Gamepad currentGamepad2 = new Gamepad();
    private Gamepad previousGamepad2 = new Gamepad();
    public ElapsedTime Timer1,Timer2,GetToSpeedTimer;
    boolean jumped1 = false, jumped2 = false;
    public Pose Push = new Pose(127,66, Math.toRadians(90)).mirror();
    public Pose Departe = new Pose(84.34,19.13,1.1941).mirror();
    private Pose Aproape = new Pose(84.64,80.09,0.87045).mirror();
    public Path GoToPush,GoToAproape,GoToDep;




    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        sorter = new Sorter(hardwareMap);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        limelight = new Limelight(hardwareMap);
        Timer1 = new ElapsedTime();
        Timer2 = new ElapsedTime();
        GetToSpeedTimer = new ElapsedTime();
        vision = new GoodLimelight(hardwareMap);
    }

    public void init_loop(){
        vision.update();
        CurrentPattern = vision.GetPattern();
        telemetry.addData("Current pattern", CurrentPattern);
        telemetry.update();
    }
    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive(true);
        outtake.Pattern = CurrentPattern;
    }

    public void BuildPaths(){
        GoToPush = new Path(new BezierLine(follower.getPose(),Push));
        GoToPush.setLinearHeadingInterpolation(follower.getHeading(),Math.toRadians(90));
        GoToAproape = new Path(new BezierLine(follower.getPose(),Aproape));
        GoToAproape.setLinearHeadingInterpolation(follower.getHeading(), Aproape.getHeading());
        GoToDep = new Path(new BezierLine(follower.getPose(),Departe));
        GoToDep.setLinearHeadingInterpolation(follower.getHeading(), Departe.getHeading());
    }
    @Override
    public void loop() {
        if(follower.getPose().getY()<70){
            outtake.AngleDep();
            Outtake.TargetRPM = 2900;
        }else {
            Outtake.TargetRPM = 2500;
            outtake.AngleApr();
        }
        vision.update();
        outtake.Update();
        intake.Update();
        outtake.OuttakeData(telemetry);

        try {
            // Copiază gamepadurile pentru comparație între frame-uri
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);


        } catch (Exception e) {
            telemetry.addLine("Exception  Assigning Gamepads. " + e);
        }

        follower.update();
        if (!automatedDrive) {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true );}




        // Dpad Up - moveP
        if (gamepad1.yWasPressed()) {
            intake.eat();
        }

        if (gamepad1.bWasPressed()) {
            intake.zero();
        }
        if(gamepad2.leftBumperWasPressed()){
            BuildPaths();
            automatedDrive = true;
            follower.followPath(GoToDep);
        }
        if(gamepad2.rightBumperWasPressed()){
            BuildPaths();
            automatedDrive = true;
            follower.followPath(GoToAproape);
        }
        if(automatedDrive && (gamepad1.left_stick_x>0.2 ||gamepad1.left_stick_y >0.2 ||!follower.isBusy())){
            automatedDrive=false;
            follower.startTeleopDrive();
        }

        if(currentGamepad1.right_bumper &&  !previousGamepad1.right_bumper){
            outtake.Request = Outtake.RequestedShoot.GREEN;
            outtake.ShootGreen();
        }
        if(currentGamepad1.left_bumper &&  !previousGamepad1.left_bumper){
            outtake.Request = Outtake.RequestedShoot.PURPLE;
            outtake.ShootPurple();
        }

        if(gamepad1.left_trigger>0.3 && !(gamepad1.right_trigger>0.05)){
            if(outtake.OneCanShootGreen()){
                outtake.StartJump1();
            }else if(outtake.TwoCanShootGreen()){
                outtake.StartJump2();
            }
        }
        if(gamepad1.right_trigger>0.3 && !(gamepad1.left_trigger>0.05)){
            if(outtake.OneCanShootPurple()){
                outtake.StartJump1();
            }else if(outtake.TwoCanShootPurple()){
                outtake.StartJump2();
            }
        }
        if(gamepad1.right_stick_button){
            outtake.stop2();
            outtake.MotorsState = Outtake.MotorState.OFF;
            outtake.Shoot1State = Outtake.ShootState.WAIT;
            outtake.Shoot2State = Outtake.ShootState.WAIT;
        }
        if(gamepad1.left_trigger>0.3 && gamepad1.right_trigger>0.3){
            outtake.ShootPattern();
        }
        if(gamepad1.leftStickButtonWasPressed()){
            outtake.motor_shooter_1.setVelocity((Outtake.TargetRPM*28)/60);
            outtake.motor_shooter_2.setVelocity((Outtake.TargetRPM*28)/60);
        }
        if(gamepad1.dpadUpWasPressed()){
            outtake.AnglerMoveForward();
        }

        if(gamepad1.dpadDownWasPressed()){
            outtake.AnglerMoveBackward();
        }

        if(gamepad1.dpadRightWasPressed()){
            outtake.AngleApr();
        }
        if(gamepad1.dpadLeftWasPressed()){
            outtake.AngleDep();
        }

        if(gamepad2.aWasPressed()){
            outtake.moveFJ1();
        }

        if(gamepad2.bWasPressed()){
            outtake.moveFJ2();
        }

        if(gamepad2.xWasPressed()){
            outtake.moveBJ1();
        }

        if(gamepad2.yWasPressed()){
            outtake.moveBJ2();
        }




        double pos2 = sorter.getPos();
        double putere = intake.intake.getPower();
        telemetry.addData("poz2: ", pos2);
        telemetry.addData("putere: ", putere);
        telemetry.addData("jumper1 ", outtake.getJ1());
        telemetry.addData("jumper2 ", outtake.getJ2());
        telemetry.addData("CurrentPattern",CurrentPattern);
        telemetry.addData("Follower X:",follower.getPose().getX());
        telemetry.addData("Follower Y:",follower.getPose().getY());
        telemetry.addData("Follower heading:",follower.getPose().getHeading());
        telemetry.addData("target rpm:",Outtake.TargetRPM);
        telemetry.update();


    }
}
