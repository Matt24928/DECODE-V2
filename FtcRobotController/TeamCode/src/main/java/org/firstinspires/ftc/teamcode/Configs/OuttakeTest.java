package org.firstinspires.ftc.teamcode.Configs;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Boss.Autos.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Pluto.SubSystems.Intake;
@Configurable

@TeleOp(name = "Outtake Test", group = "Configs")
public class OuttakeTest extends OpMode {

    public DcMotorEx shooter1,shooter2;

    public static double highVelocity = 2100;
    public static double lowVelocity = 1100;
    Outtake outtake;
    Intake intake;

    double curTargetVelocity = highVelocity;

    double F = 0;
    double P = 0;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};

    int stepIndex = 1;

    @Override
    public void init() {
        shooter1 = hardwareMap.get(DcMotorEx.class, "Motor_left");
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter2 = hardwareMap.get(DcMotorEx.class, "Motor_right");
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("Init complete yay");
        intake = new Intake(hardwareMap);
    }

    @Override
    public void loop(){
        if(gamepad1.leftBumperWasPressed()){
            intake.Eat();
        }
        if(gamepad1.rightBumperWasPressed()){
            intake.Stop();
        }
        if(gamepad1.yWasPressed()){
            if(curTargetVelocity == highVelocity){
                curTargetVelocity = lowVelocity;
            } else {
                curTargetVelocity = highVelocity;
            }
        }

        if(gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1)% stepSizes.length;
        }

        if(gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }

        if(gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }

        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }

        if(gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }
        if(gamepad2.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }

        if(gamepad2.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }

        if (gamepad2.leftBumperWasPressed()) {
            P += stepSizes[stepIndex];
        }

        if(gamepad2.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }


        //Setting new coefficients
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        //Setting our velocity
        shooter1.setVelocity(curTargetVelocity);
        shooter2.setVelocity(curTargetVelocity);

        double curVelocity = shooter1.getVelocity();
        double error = curTargetVelocity - curVelocity;
        double curVelocity2 = shooter2.getVelocity();
        double error2 = curTargetVelocity- curVelocity2;
        double current1 = shooter1.getCurrent(CurrentUnit.MILLIAMPS);
        double current2 = shooter2.getCurrent(CurrentUnit.MILLIAMPS);


        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", curVelocity);
        telemetry.addData("Current Velocity", "%.2f", curVelocity2);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addLine();
        telemetry.addData("Tuning P", "%.4f (D-Pad Up or Down)", P);
        telemetry.addData("Tuning F", "%.4f (D-Pad Left or Right)", F);
        telemetry.addData("Step Size", "%.4f (B button)", stepSizes[stepIndex]);
        telemetry.addData("current1",current1);
        telemetry.addData("current2",current2);
        telemetry.addData("mode 1",shooter1.isMotorEnabled());
        telemetry.addData("mode 2",shooter2.isMotorEnabled());

    }

}