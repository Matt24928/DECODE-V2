package org.firstinspires.ftc.teamcode.Pluto.Teleops;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Commands.TimerCommand;
import org.firstinspires.ftc.teamcode.Pluto.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.Pluto.SubSystems.Limelight;
import org.firstinspires.ftc.teamcode.Pluto.SubSystems.Outtake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.opencv.core.Mat;

@TeleOp
@Configurable
public class PlutoTeleOp extends CommandOpMode {

    private Limelight limelight;
    private Intake intake;
    private Outtake outtake;
    private Follower follower;
    private GamepadEx driver;
    private SequentialCommandGroup Attack_Goal;
    PIDFController headingPID;
    boolean AutoShoot = false;
    boolean autoHeading = false;
    double rotation = 0;
    public static PIDFCoefficients pidfCoefficients = new PIDFCoefficients(0.85,0,0.075,0.05);

    @Override
    public void initialize() {
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        limelight = new Limelight(hardwareMap);
        limelight.limelight.pipelineSwitch(3);
        follower.startTeleopDrive(false); //daca dam pe true cam zboara cred
        driver = new GamepadEx(gamepad1);
        headingPID = new PIDFController(pidfCoefficients);

        Attack_Goal = new SequentialCommandGroup(
                new InstantCommand(intake::Eat),
                new TimerCommand(1000),
                new InstantCommand(outtake::Down)

        );

        new GamepadButton(driver, GamepadKeys.Button.DPAD_LEFT).whenPressed(outtake::Up);
        new GamepadButton(driver, GamepadKeys.Button.DPAD_RIGHT).whenPressed(outtake::Down);
        new GamepadButton(driver, GamepadKeys.Button.DPAD_UP).whenPressed(outtake::AnglerP);
        new GamepadButton(driver, GamepadKeys.Button.DPAD_DOWN).whenPressed(outtake::AnglerM);
        new GamepadButton(driver, GamepadKeys.Button.Y).whenPressed(Attack_Goal);
        new GamepadButton(driver, GamepadKeys.Button.B).whenPressed(intake::Stop);
        new GamepadButton(driver, GamepadKeys.Button.X).whenPressed(outtake::Shoot);
        new GamepadButton(driver, GamepadKeys.Button.A).whenPressed(outtake::Stop);
        //new GamepadButton(driver, GamepadKeys.Button.LEFT_BUMPER).whenPressed(outtake::Shoot_Dep);
        new Trigger(()->driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>0.3)
                .whenActive(()->{outtake.decrease_vel();});
        new Trigger(()->driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>0.3)
                .whenActive(()->{outtake.increase_vel();});

        register(outtake,intake,limelight);
    }

    @Override
    public void run(){
        super.run();
        follower.update();
        headingPID.setCoefficients(pidfCoefficients);
        limelight.periodic();
        if(gamepad1.optionsWasPressed()){
            AutoShoot = !AutoShoot;
        }
        double ta = limelight.getTa();
        double angle,shoot_velocity,distance;
        distance = MathFunctions.clamp(74.11189-37.98231*Math.log(ta),30,180);
        if(limelight.hasTarget()){
            shoot_velocity = MathFunctions.clamp(0.000959713*(Math.pow(distance,3))-0.185345*Math.pow(distance,2)+15.31945*distance+624.43113,900,1600);
            angle = MathFunctions.clamp(0.00000143953*Math.pow(distance,3)-0.000397956*Math.pow(distance,2)+0.0381062*distance-0.368977,0.6,1);
            if(distance>100){
              shoot_velocity = shoot_velocity*1.05;
              angle = 1;
            }

        }else{
            shoot_velocity = 1000;
            angle = 0.8; // la ghici
        }




        if(AutoShoot){
            outtake.ShootVelocity(shoot_velocity);
            outtake.ShootAngle(angle);
        }
        if(gamepad1.touchpadWasPressed()){
            autoHeading = !autoHeading;
            headingPID.reset();
            gamepad1.setLedColor(74,236,16,400);
        }
        if (gamepad1.right_bumper && limelight.hasTarget()) {

            double txRad = Math.toRadians(limelight.getTyDeg());

            if (Math.abs(txRad) < Math.toRadians(0.05)) {
                headingPID.updateError(0);
            } else {
                headingPID.updateError(txRad);
            }

            rotation = headingPID.run();

            if (Math.abs(rotation) < 0.05) {
                rotation = 0;
            }

        } else {
            headingPID.reset();
            rotation = -gamepad1.right_stick_x;
        }
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                rotation,
                true
        );
        if(intake.intake_1.getCurrent(CurrentUnit.MILLIAMPS)>2100&&intake.intake_2.getCurrent(CurrentUnit.MILLIAMPS)>7500 && gamepad1.left_bumper){
            intake.Stop();
        }
        if(gamepad1.right_stick_button){
            intake.Spit();
        }
//        if(outtake.TicksMed!= 0 &&outtake.TicksMed>= outtake.TicksMed-50 && outtake.TicksMed <= outtake.TicksMed + 50){
//            gamepad1.rumble(100);
//        }
        outtake.OuttakeData(telemetry);
        intake.IntakeData(telemetry);
        telemetry.addData("Ty:",limelight.getTyDeg());
        telemetry.addData("Ta:",limelight.getTa());
        telemetry.addData("Pose 3D:",limelight.GetDistance());
        telemetry.addData("Distance: ",distance);
        telemetry.update();
    }
}
