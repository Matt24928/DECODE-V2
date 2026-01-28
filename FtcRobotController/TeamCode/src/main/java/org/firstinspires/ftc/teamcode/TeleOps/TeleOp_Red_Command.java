package org.firstinspires.ftc.teamcode.TeleOps;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.util.Timing;
import com.bylazar.gamepad.Gamepad;
import com.bylazar.gamepad.GamepadPluginConfig;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Autos.AutoBOSS_RED;
import org.firstinspires.ftc.teamcode.Commands.TimerCommand;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.OuttakeCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;
import java.util.Timer;
import java.util.concurrent.TimeUnit;
@TeleOp
public class TeleOp_Red_Command extends CommandOpMode {
    private OuttakeCommand outtake;
    private Intake intake;
    private GamepadEx driverGamepad;
    private Follower follower;
    private List<LynxModule> allHubs;
    private final Pose startPose = new Pose(125.2, 120, Math.toRadians(43));
    private SequentialCommandGroup ShootAll,Shoot1,Shoot2;
    private ParallelCommandGroup Ceva,JumpAll;
    private InstantCommand Eat,Stop,Spit,Lock;
    PIDFController controller;
    boolean headingLock = true;
    double headingGoal = Math.toRadians(180);


    private final Pose Shoot_Far = new Pose(85, 17);
    private final Pose startt = new Pose(107, 91);
    private boolean automatedDrive = false;

   // private Path Shoot_From_Far;

    private void snapTo(double headingRad){
        headingGoal = headingRad;
        headingLock = true;
        controller.reset();
    }



    @Override
    public void initialize() {

        outtake = new OuttakeCommand(hardwareMap);
        intake = new Intake(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        allHubs = hardwareMap.getAll(LynxModule.class);
        follower.setStartingPose(startPose);
        follower.startTeleopDrive(false);
        driverGamepad = new GamepadEx(gamepad1);
        outtake.targetRpm = 1100;


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
            outtake.targetRpm = 1100;
            outtake.SetAngle(0.28);}

        );
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_LEFT).whenPressed(()->{
            outtake.targetRpm = 1350;
            outtake.SetAngle(0.25);}
        );


        telemetry.addData("anglerpoz", outtake::getAnglerPoz);
        telemetry.update();


        register(outtake,intake);
    }


    @Override
    public void run() {

        super.run();
        follower.update();

        if(outtake.rpmmed>= outtake.targetRpm-70 && outtake.rpmmed <= outtake.targetRpm + 70){
            gamepad1.rumble(100);
        }





        follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // field-centric
        );





        outtake.OuttakeData(telemetry);

        for (LynxModule module : allHubs) {
            telemetry.addData("Bulk mode", module.getBulkCachingMode().toString());
        }
        telemetry.addLine();
        telemetry.addLine("Asta te intereseaza ");
        telemetry.addData("TargetTicks ", outtake.targetRpm);
        telemetry.addData("Current Ticks",outtake.getCurrentTicks());
//        telemetry.addData("pose",follower.getPose());
        telemetry.update();
    }

}
