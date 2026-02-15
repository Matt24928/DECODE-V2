package org.firstinspires.ftc.teamcode.Boss.Autos.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake extends SubsystemBase {
    public DcMotor intake;
    public static ElapsedTime IntakeTimer;
    double Eat = 1,spit;
    public enum IntakeStates{
        EATING,
        SPITING,
        PUSHING,
        IDLE
    }
    public static IntakeStates intakeStates = IntakeStates.IDLE;

    public Intake(HardwareMap hw){
        IntakeTimer = new ElapsedTime();
        intake = hw.get(DcMotor.class,"Intake_m");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setPower(0);
    }
     public void eat(){
        intakeStates = IntakeStates.EATING;
        intake.setPower(1);
     }
     public void spit(){
        intakeStates = IntakeStates.PUSHING;
        intake.setPower(-1);
     }
     public void push(){
        IntakeTimer.reset();
        intakeStates = IntakeStates.PUSHING;
     }

     public void zero(){
        intakeStates = IntakeStates.IDLE;
        intake.setPower(0);
        intake.getZeroPowerBehavior();
     }
     public void Update(){
        if(intakeStates == IntakeStates.PUSHING && IntakeTimer.seconds()<2){
            intake.setPower(1);
        }
        if(intakeStates == IntakeStates.PUSHING && IntakeTimer.seconds()>2){
            intake.setPower(0);
        }
        if(intakeStates == IntakeStates.IDLE && (intake.getPower()!=0)){
            intake.setPower(0);
        }
        if(intakeStates == IntakeStates.EATING &&(intake.getPower()!=1)){
            intake.setPower(1);
        }
        if(intakeStates == IntakeStates.SPITING && (intake.getPower()!=-1)){
            intake.setPower(-1);
        }

     }

}
