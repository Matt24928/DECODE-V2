package org.firstinspires.ftc.teamcode.Boss.Autos.Subsystems;

import android.graphics.Color;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Configurable
public class OuttakeCommand extends SubsystemBase {
     public double ANG_departe = 0.17;
    static double ANG_aproape = 0.18, ANG_aproape2 = 0.3;
    VoltageSensor volt;
    NormalizedColorSensor colorSensor1;
    NormalizedColorSensor colorSensor2;
    DcMotorEx motor_shooter_1,motor_shooter_2;
    Servo jumper1,jumper2,angler;
    final double P1 = 159.4,F1=12.6;//159.4 12.6
    final double LOW1 = 0.7,JUMP1 = 0.43;
    final double LOW2 = 0.17,JUMP2 = 0.39;
    final double k1=1.03,k2=1.03,r=0.048;
    public float hue1,hue2,sat1,sat2,val1,val2;
    public double current1,currentAlert1,Ticks1,RPM1,AngularSpeed1,LiniarSpeed1;
    public double current2,currentAlert2,Ticks2,RPM2,AngularSpeed2,LiniarSpeed2;
    double anglePoz;
    double distance1,distance2;
    Outtake.DetectedColor color1,color2;
    public double RPM,rpmmed;
    public double targetRpm = 1300; //e defapt tick per second
    public OuttakeCommand(HardwareMap hw){
        motor_shooter_1 = hw.get(DcMotorEx.class, "MotorShooter_2");
        motor_shooter_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_shooter_1.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients1 = new PIDFCoefficients(P1,0,0,F1);
        motor_shooter_1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients1);
        motor_shooter_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        motor_shooter_2 = hw.get(DcMotorEx.class, "MotorShooter_1");
        motor_shooter_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_shooter_2.setDirection(DcMotorSimple.Direction.FORWARD);
        motor_shooter_2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients1);
        motor_shooter_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        jumper1 = hw.servo.get("Jumper_1");
        jumper2 = hw.servo.get("Jumper_2");
        angler = hw.servo.get("Angler");

        volt = hw.voltageSensor.iterator().next();
        colorSensor1 = hw.get(NormalizedColorSensor.class,"SensorColor_1");
        colorSensor2 = hw.get(NormalizedColorSensor.class, "SensorColor_2");
        colorSensor1.setGain(2f);
        colorSensor2.setGain(2f);

        jumper1.setPosition(LOW1);
        jumper2.setPosition(LOW2);
        angler.setPosition(ANG_aproape);
    }

    public void OuttakeData(Telemetry t){
        current1 = motor_shooter_1.getCurrent(CurrentUnit.MILLIAMPS);
        currentAlert1 = motor_shooter_1.getCurrentAlert(CurrentUnit.MILLIAMPS);
        Ticks1 = motor_shooter_1.getVelocity();
        RPM1 = (Ticks1/28)*60;
        AngularSpeed1 = motor_shooter_1.getVelocity(AngleUnit.RADIANS);
        LiniarSpeed1 = k1*AngularSpeed1*r;

        t.addLine("--Motor 1--");
        t.addData("Current (mA)", "%.1f", current1);
        t.addData("Alert Current (mA)", "%.1f", currentAlert1);
        t.addData("Ticks/sec", "%.1f", Ticks1);
        t.addData("RPM", "%.1f", RPM1);
        t.addData("Angular Speed (rad/s)", "%.2f", AngularSpeed1);
        t.addData("Linear Speed (m/s)", "%.2f", LiniarSpeed1);


        current2 = motor_shooter_2.getCurrent(CurrentUnit.MILLIAMPS);
        currentAlert2 = motor_shooter_2.getCurrentAlert(CurrentUnit.MILLIAMPS);
        Ticks2 = motor_shooter_2.getVelocity();
        RPM2 = (Ticks2/28)*60;
        AngularSpeed2 = motor_shooter_2.getVelocity(AngleUnit.RADIANS);
        LiniarSpeed2 = k2*AngularSpeed2*r;

        t.addLine("--Motor 2--");
        t.addData("Current (mA)", "%.1f", current2);
        t.addData("Alert Current (mA)", "%.1f", currentAlert2);
        t.addData("Ticks/sec", "%.1f", Ticks2);
        t.addData("RPM", "%.1f", RPM2);
        t.addData("Angular Speed (rad/s)", "%.2f", AngularSpeed2);
        t.addData("Linear Speed (m/s)", "%.2f", LiniarSpeed2);

        color1 = getDetectedColor1(t);
        color2 = getDetectedColor2(t);

//        t.addData("Sensor 1", "%s", color1);
//        t.addData("Sensor 2", "%s", color2);

        t.addData("Shooter Poz","%f",anglePoz);
//        t.addData("Jumper1","%f",jumper1.getPosition());
//        t.addData("Jumper1","%f",jumper2 .getPosition());


        t.addData("Current Angler Pos:", angler.getPosition());
        RPM = (RPM1+RPM2)/2;
        rpmmed = (Ticks1+Ticks2)/2;
        t.addData("RPM mediu:", RPM);
        t.addData("Ticks med:",rpmmed);


    }
    public static double flywheelSpeed(double goadDist){
        return (MathFunctions.clamp(0.00953626*Math.pow(goadDist,2) + 2.27592*goadDist + 893.53127,0,1400));
    };
    public static double hoodAngle(double goalDist){
        return MathFunctions.clamp(1.53007-0.264419*Math.log(goalDist),0.05,0.6);
   };
    private Outtake.DetectedColor detectColor(
            NormalizedColorSensor sensor,
            Telemetry telemetry,
            String name
    ) {
        NormalizedRGBA colors = sensor.getNormalizedColors();
        float[] hsv = new float[3];
        Color.colorToHSV(colors.toColor(), hsv);

        float hue = hsv[0];
        float sat = hsv[1];
        float val = hsv[2];

        telemetry.addLine("--" + name + "--");
        telemetry.addData("Hue_", hue);
        telemetry.addData("Sat_", sat);
        telemetry.addData("Val_", val);

        if (hue > 90 && hue < 180)
            return Outtake.DetectedColor.GREEN;
        else if (hue > 205 && hue < 295)
            return Outtake.DetectedColor.PURPLE;
        else return Outtake.DetectedColor.UNKNOWN;
    }
    public Outtake.DetectedColor getDetectedColor1(Telemetry t){
        return detectColor(colorSensor1, t, "Sensor 1");
    }
    public Outtake.DetectedColor getDetectedColor2(Telemetry t){
        return detectColor(colorSensor2, t, "Sensor 2");
    }
    public void Low2(){
        jumper2.setPosition(LOW2);
    }
    public void Jump2() {
        jumper2.setPosition(JUMP2);
    }
    public void Low1(){
        jumper1.setPosition(LOW1);
    }
    public void Jump1() {
        jumper1.setPosition(JUMP1);
    }
    public void Shoot() {
        motor_shooter_1.setVelocity(1630);
        motor_shooter_2.setVelocity(1630);
    }
    public void ShootApr(){
        motor_shooter_1.setVelocity(targetRpm);
        motor_shooter_2.setVelocity(targetRpm);
    }
    public void autoSpeed(double speed){
        motor_shooter_1.setVelocity(speed);
        motor_shooter_2.setVelocity(speed);
    }
    public void stop2(){
        motor_shooter_1.setPower(0);
        motor_shooter_2.setPower(0);
    }
    public void slowdown(){
        motor_shooter_1.setVelocity(targetRpm);
        motor_shooter_2.setPower(0);
    }
    public void AngleApr(){
        angler.setPosition(ANG_aproape);
    }
    public void AngleApr2(){
        angler.setPosition(ANG_aproape2);
    }
    public void AngleDep(){
        angler.setPosition(ANG_departe);
    }
    public void SetAngle(double angle){
        angler.setPosition(angle);
    }
    public void moveF() { angler.setPosition(angler.getPosition()+0.05);}
    public void moveB() { angler.setPosition(angler.getPosition()-0.05);}
    public double getAnglerPoz() {
       return  angler.getPosition();
    }
    public double getCurrentTicks(){
        return motor_shooter_2.getVelocity();
    }

    public void moveFJ1() {
        jumper1.setPosition(jumper1.getPosition()+ 0.01);
    }

    public void moveBJ1() {
        jumper1.setPosition(jumper1.getPosition()- 0.01);
    }

    public void moveFJ2() {
        jumper2.setPosition(jumper2.getPosition()+ 0.01);
    }

    public void moveBJ2() {
        jumper2.setPosition(jumper2.getPosition()- 0.01);
    }
}
