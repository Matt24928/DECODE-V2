package org.firstinspires.ftc.teamcode.Boss.Autos.TeleOps;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Boss.Autos.AutoBOSS_BLUE;
import org.firstinspires.ftc.teamcode.Boss.Autos.AutoBOSS_FAR_BLUE_GOOD;
import org.firstinspires.ftc.teamcode.Boss.Autos.AutoBOSS_FAR_RED;
import org.firstinspires.ftc.teamcode.Boss.Autos.AutoBOSS_RED;
import org.firstinspires.ftc.teamcode.Commands.TimerCommand;
import org.firstinspires.ftc.teamcode.Boss.Autos.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Boss.Autos.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Boss.Autos.Subsystems.OuttakeCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@TeleOp
@Configurable
@Disabled
//asta e teleopul principal merge la ambele aliante
public class TeleOp_Red_Command extends CommandOpMode {

    private Limelight limelight;
    private OuttakeCommand outtake;
    private Intake intake;
    private GamepadEx driverGamepad;
    private Follower follower;
    private List<LynxModule> allHubs;
    private Pose startPose;
    private final Pose RedGoal = new Pose(138,138);
    private SequentialCommandGroup ShootAll,Shoot1,Shoot2;
    private ParallelCommandGroup Ceva,JumpAll;
    private InstantCommand Eat,Stop,Spit,Lock;

    boolean isAutoShoot = false;
    public PathChain HeadingLock;
    private final Pose Shoot_Far = new Pose(85, 17);
    private final Pose startt = new Pose(107, 91);
    private boolean automatedDrive = false;
    private boolean slow = false;
    public double distance,distaceByTa;

   // private Path Shoot_From_Far;

    double targetHeading ;
    boolean headingLock = true;
    public static PIDFCoefficients pidfCoefficients = new PIDFCoefficients(1,0,0.1,0);
    PIDFController headingPID;

    boolean autoHeading = false;
    double rotation = 0;


    @Override
    public void initialize() {
        if(AutoBOSS_RED.LastPose!=null){
            startPose = AutoBOSS_RED.LastPose;
        }else if(AutoBOSS_FAR_RED.LastPose!=null){
            startPose = AutoBOSS_FAR_RED.LastPose;
        }else if(AutoBOSS_BLUE.LastPose!=null){
            startPose = AutoBOSS_BLUE.LastPose;
        }else if(AutoBOSS_FAR_BLUE_GOOD.LastPose!=null){
            startPose = AutoBOSS_FAR_BLUE_GOOD.LastPose;
        }else {
            startPose = new Pose(56, 8, Math.toRadians(90));
        }

        outtake = new OuttakeCommand(hardwareMap);
        intake = new Intake(hardwareMap);
        limelight = new Limelight(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        allHubs = hardwareMap.getAll(LynxModule.class);
        follower.setStartingPose(startPose);
        follower.startTeleopDrive(false);
        driverGamepad = new GamepadEx(gamepad1);
        outtake.targetRpm = 1100;
        headingPID = new PIDFController(pidfCoefficients);




        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        Ceva = new ParallelCommandGroup(
                new InstantCommand(outtake::Jump2),
                new InstantCommand(intake::eat)

        );
        JumpAll = new ParallelCommandGroup(
                new InstantCommand(outtake::Jump1),
                new InstantCommand(outtake::Jump2)
        );
        ShootAll = new SequentialCommandGroup(
                new InstantCommand(outtake::Jump1),
                new TimerCommand(200),
                new InstantCommand(outtake::Low1),
                new TimerCommand(350),
                Ceva,
                new TimerCommand(200),
                new InstantCommand(outtake::Low2),
                new TimerCommand(450),
                JumpAll,
                new TimerCommand(100),
                new InstantCommand(outtake::Low1),
                new InstantCommand(outtake::Low2)
        );
        Shoot1 = new SequentialCommandGroup(
                new InstantCommand(outtake::Jump1),
                new TimerCommand(200),
                new InstantCommand(outtake::Low1)
        );
        Shoot2 = new SequentialCommandGroup(
                new InstantCommand(outtake::Jump2),
                new TimerCommand(200),
                new InstantCommand(outtake::Low2)
        );




        new GamepadButton(driverGamepad, GamepadKeys.Button.Y).whenPressed(intake::eat);
        new GamepadButton(driverGamepad, GamepadKeys.Button.B).whenPressed(intake::zero);
        new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER).whenPressed(Shoot1);
        new GamepadButton(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER).whenPressed(Shoot2);
        new GamepadButton(driverGamepad, GamepadKeys.Button.X).whenPressed(outtake::ShootApr);
        new GamepadButton(driverGamepad, GamepadKeys.Button.A).whenPressed(outtake::stop2);
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_UP).whenPressed(outtake::moveF);
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_DOWN).whenPressed(outtake::moveB);
        new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(intake::spit);
       // new GamepadButton(driverGamepad,GamepadKeys.Button.DPAD_LEFT).whenPressed(ShootAll);
      // new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_LEFT).whenPressed(()->snapTo(headingGoal));
        new Trigger(()->driverGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>0.3)
                .whenActive(()->{outtake.targetRpm -= 50;});
        new Trigger(()->driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>0.3)
                .whenActive(()->{outtake.targetRpm += 50;});
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_RIGHT).whenPressed(()->{
                    if(isAutoShoot==true){
                        isAutoShoot=false;
                    }else {
                        isAutoShoot=true;
                    }
                }
        );
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(() -> {
                    autoHeading = !autoHeading;
                    headingPID.reset();
                });


        telemetry.addData("anglerpoz", outtake::getAnglerPoz);
        telemetry.update();


        register(outtake,intake,limelight);
    }
    public double GetDistaceByTa(double Ta){
        return MathFunctions.clamp(67.15179*Math.pow(Ta,-0.571858),20,150);
    };

    @Override
    public void run() {

        super.run();
        headingPID.setCoefficients(pidfCoefficients);
        if(gamepad1.touchpadWasPressed()){
            autoHeading = !autoHeading;
            headingPID.reset();
            gamepad1.setLedColor(74,236,16,400);
        }
        if(gamepad1.psWasPressed()){
            if(isAutoShoot==true){
                isAutoShoot=false;
            }else {
                isAutoShoot=true;
            }
            gamepad1.setLedColor(207,16,236,400);
        }
        limelight.periodic();
        follower.update();
        headingPID.setCoefficients(pidfCoefficients);
        if (isAutoShoot) {
            outtake.autoSpeed(OuttakeCommand.flywheelSpeed(distaceByTa*1.07));
            outtake.targetRpm = OuttakeCommand.flywheelSpeed(distaceByTa*1.07);
            outtake.SetAngle(OuttakeCommand.hoodAngle(distaceByTa*0.96));

        }

//        targetHeading = Math.atan2(
//                RedGoal.getY() - follower.getPose().getY(),
//                RedGoal.getX() - follower.getPose().getX()
//        );
        if(gamepad1.optionsWasPressed()){
            slow = !slow;
        }
        if(outtake.rpmmed>= outtake.targetRpm-70 && outtake.rpmmed <= outtake.targetRpm + 70){
            gamepad1.rumble(100);
        }
        if (autoHeading && limelight.hasTarget()) {

            double txRad = Math.toRadians(limelight.getTxDeg());

            if (Math.abs(txRad) < Math.toRadians(0.3)) {
                headingPID.updateError(0);
            } else {
                headingPID.updateError(-txRad);
            }

            rotation = headingPID.run();

            if (Math.abs(rotation) < 0.1) {
                rotation = 0;
            }

        } else {
            headingPID.reset();
            rotation = -gamepad1.right_stick_x;
        }
        if(!slow){
        follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    rotation,
                    true
        );}else {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    rotation*0.5,
                    true);
        };

        double Ta = limelight.getTa();
        distaceByTa = GetDistaceByTa(Ta);
        distance = Math.sqrt((RedGoal.getX()-follower.getPose().getX())*(RedGoal.getX()-follower.getPose().getX())+(RedGoal.getY()-follower.getPose().getY())*(RedGoal.getY()-follower.getPose().getY()));
        outtake.OuttakeData(telemetry);


        for (LynxModule module : allHubs) {
            telemetry.addData("Bulk mode", module.getBulkCachingMode().toString());
        }
        telemetry.addLine();
        telemetry.addLine("Asta te intereseaza ");
        telemetry.addData("TargetTicks ", outtake.targetRpm);
        telemetry.addData("Current Ticks",outtake.getCurrentTicks());
        telemetry.addData("distanta sper",distance);
        telemetry.addData("pose",follower.getPose());
        telemetry.addData("Ta: ",Ta);
        telemetry.update();

    }

}
