package org.firstinspires.ftc.teamcode.Pluto.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
@Configurable
public class Outtake extends SubsystemBase {

    DcMotorEx MotorLeft,MotorRight;
    Servo left,right,angler;
    double shoot_velocity;
    double p=95, f=17.9;
    public static double vel = 1600;
    double LOW1, LOW2, UP1, UP2;
    public double current1,currentAlert1,Ticks1,RPM1,AngularSpeed1,LiniarSpeed1,TicksMed;
    public double current2,currentAlert2,Ticks2,RPM2,AngularSpeed2,LiniarSpeed2;
    public Outtake(HardwareMap hw){
        MotorLeft = hw.get(DcMotorEx.class,"Motor_left");
        MotorLeft.setDirection(DcMotorSimple.Direction.REVERSE); //maybe
        MotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(p,0,0,f);
        MotorLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);

        MotorRight = hw.get(DcMotorEx.class,"Motor_right");
        MotorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);

        MotorRight.setPower(0);
        MotorLeft.setPower(0);

        MotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        MotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        left = hw.servo.get("Servo_l");
        right = hw.servo.get("Servo_r");
        left.setDirection(Servo.Direction.REVERSE);
        right.setPosition(0.5);
        left.setPosition(0.5);

        angler = hw.servo.get("Angler");
        angler.setDirection(Servo.Direction.REVERSE);

    }
    public void AnglerP(){
        angler.setPosition(angler.getPosition()+0.05);
    }
    public void AnglerM(){
        angler.setPosition(angler.getPosition()-0.05);
    }
    public void Shoot(){
        MotorLeft.setVelocity(vel);
        MotorRight.setVelocity(vel);
    }
    public void Stop(){
        MotorLeft.setVelocity(0);
        MotorRight.setVelocity(0);
    }
    public void Down(){
        left.setPosition(0.59);
        right.setPosition(0.59);
    }
    public void Up(){
        left.setPosition(0.8);
        right.setPosition(0.8);
    }

    public void MoveUp(){
        left.setPosition(left.getPosition() + 0.03);
        right.setPosition(right.getPosition() + 0.03);

    }

    public void MoveDown(){
        left.setPosition(left.getPosition() - 0.03);
        right.setPosition(right.getPosition() - 0.03);

    }

    public void increase_vel(){
        vel+=50;
    }

    public void decrease_vel(){
        vel-=50;
    }


    public void OuttakeData(Telemetry t){
        current1 = MotorLeft.getCurrent(CurrentUnit.MILLIAMPS);
        Ticks1 = MotorLeft.getVelocity();
        current2 = MotorRight.getCurrent(CurrentUnit.MILLIAMPS);
        Ticks2 = MotorRight.getVelocity();
        TicksMed = (Ticks1+Ticks2)/2;
        t.addData("velocity: ", vel);
        t.addLine("--Motor 1--");
        t.addData("Current (mA)", "%.1f", current1);
        t.addData("Ticks/sec", "%.1f", Ticks1);
        t.addData("poz: ",left.getPosition());
        t.addLine("--Motor 2--");
        t.addData("Current (mA)", "%.1f", current2);
        t.addData("Ticks/sec", "%.1f", Ticks2);
        t.addLine("--Others--");
        t.addData("Medium Ticks :",TicksMed);
        t.addData("poz left",left.getPosition());
        t.addData("poz right",right.getPosition());
        t.addData("Angler: ",angler.getPosition());
    }



}
