package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class TeleOp1 extends OpMode {

    DcMotor front_right, front_left, back_right, back_left;

    @Override
    public void init() {
        front_right = hardwareMap.get(DcMotor.class, "motor_rf");
        front_left = hardwareMap.get(DcMotor.class, "motor_lf");
        back_right = hardwareMap.get(DcMotor.class, "motor_rb");
        back_left = hardwareMap.get(DcMotor.class, "motor_lb");
    }

    @Override
    public void loop() {
        if(gamepad1.aWasPressed()){
            front_right.setPower(1);
        }
        if(gamepad1.triangleWasPressed()){
            front_left.setPower(1);
        }
        if(gamepad1.bWasPressed()){
            back_right.setPower(1);
        }
        if(gamepad1.xWasPressed()){
            back_left.setPower(1);
        }

        if(gamepad1.dpadUpWasPressed()){
            front_left.setPower(0);
        }
        if(gamepad1.dpadDownWasPressed()){
            front_right.setPower(0);
        }

        if(gamepad1.dpadRightWasPressed()){
            back_right.setPower(0);
        }
        if(gamepad1.dpadLeftWasPressed()){
            back_left.setPower(0);
        }
    }
}
