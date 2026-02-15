package org.firstinspires.ftc.teamcode.Pluto.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake extends SubsystemBase {
    DcMotor intake_1, intake_2;
    boolean change = false;

    public Intake(HardwareMap hw){
        intake_1 = hw.get(DcMotor.class,"Intake_jos");
        intake_2 = hw.get(DcMotor.class, "Intake_sus");


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
        intake_2.setPower(0.7);
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
}
