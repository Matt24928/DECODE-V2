package org.firstinspires.ftc.teamcode.Pluto.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Intake extends SubsystemBase {
    public DcMotorEx intake_1, intake_2;
    boolean change = false;

    public Intake(HardwareMap hw){
        intake_1 = hw.get(DcMotorEx.class,"Intake_jos");
        intake_2 = hw.get(DcMotorEx.class, "Intake_sus");


        intake_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake_1.setPower(0);
        intake_2.setPower(0);


    }

//    public void eat_or_spit(){
//        change=!change;
//        if(!change) {
//            intake_1.setPower(1);
//            intake_2.setPower(1);
//        }
//        else {
//            intake_1.setPower(0);
//            intake_+2.setPower(0);
//        }
//    }
    public void Eat(){
        intake_1.setPower(1);
        intake_2.setPower(1);
    }

    public void Eat_mai_incet(){
        intake_1.setPower(0.9);
        intake_2.setPower(0.9);
    }
    public void StopSus(){
        intake_2.setPower(0);
    }
    public void UndoSus(){
        intake_2.setPower(-0.15);
    }
    public void Stop(){
        intake_1.setPower(0);
        intake_2.setPower(0);
    }

    public void Spit(){
        intake_1.setPower(-1);
        intake_2.setPower(-1);
    }
    public void IntakeData(Telemetry telemetry){
        telemetry.addData("Current 1 ",intake_1.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("Current 2 ",intake_2.getCurrent(CurrentUnit.MILLIAMPS));
    }
}
