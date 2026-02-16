package org.firstinspires.ftc.teamcode.Pluto.Teleops;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Pluto.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.Pluto.SubSystems.Limelight;
import org.firstinspires.ftc.teamcode.Pluto.SubSystems.Outtake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@TeleOp
@Configurable
public class PlutoTeleOp extends CommandOpMode {

    private Limelight limelight;
    private Intake intake;
    private Outtake outtake;
    private Follower follower;
    private GamepadEx driver;

    @Override
    public void initialize() {
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        limelight = new Limelight(hardwareMap);
        follower.startTeleopDrive(false); //daca dam pe true cam zboara cred
        driver = new GamepadEx(gamepad1);

        new GamepadButton(driver, GamepadKeys.Button.DPAD_LEFT).whenPressed(outtake::Up);
        new GamepadButton(driver, GamepadKeys.Button.DPAD_RIGHT).whenPressed(outtake::Down);
        new GamepadButton(driver, GamepadKeys.Button.DPAD_UP).whenPressed(outtake::AnglerP);
        new GamepadButton(driver, GamepadKeys.Button.DPAD_DOWN).whenPressed(outtake::AnglerM);
        new GamepadButton(driver, GamepadKeys.Button.Y).whenPressed(intake::Eat);
        new GamepadButton(driver, GamepadKeys.Button.B).whenPressed(intake::Stop);
        new GamepadButton(driver, GamepadKeys.Button.X).whenPressed(outtake::Shoot);
        new GamepadButton(driver, GamepadKeys.Button.A).whenPressed(outtake::Stop);
        new GamepadButton(driver, GamepadKeys.Button.LEFT_BUMPER).whenPressed(outtake::Shoot_Dep);
        new GamepadButton(driver, GamepadKeys.Button.RIGHT_BUMPER).whenPressed(outtake::MoveUp);
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

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true // Robot Centric
        );
//        if(outtake.TicksMed!= 0 &&outtake.TicksMed>= outtake.TicksMed-50 && outtake.TicksMed <= outtake.TicksMed + 50){
//            gamepad1.rumble(100);
//        }
        outtake.OuttakeData(telemetry);
        telemetry.addData("Ty:",limelight.getTyDeg());
        telemetry.addData("Ta:",limelight.getTa());
        telemetry.addData("Pose 3D:",limelight.GetDistance());
        telemetry.update();
    }
}
