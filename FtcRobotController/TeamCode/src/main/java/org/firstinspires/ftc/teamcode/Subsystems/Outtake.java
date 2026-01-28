package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeTimer;

import android.graphics.Color;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.Autos.AutoRed_1;

import java.util.Base64;
import java.util.TreeMap;

@Configurable
public class Outtake extends SubsystemBase {
    public DcMotorEx motor_shooter_1,motor_shooter_2;
    final double k1=1.03,k2=1.03,r=0.048;
    double batteryVoltage;
    VoltageSensor volt;
    NormalizedColorSensor colorSensor1;
    NormalizedColorSensor colorSensor2;
    Servo jumper1,jumper2,angler;
    final double P1 = 15,P2 = 15,F1=17.1020,F2=17.1020;
    final double LOW1 = 0.7,JUMP1 = 0.34;
    final double LOW2 = 0.2,JUMP2 = 0.45;
    public double WAIT = 2;
    //please baga valoare
    //HAHAHAHA DOUBLE JUMP, GET IT?
    public float hue1,hue2,sat1,sat2,val1,val2;
    public double current1,currentAlert1,Ticks1,RPM1,AngularSpeed1,LiniarSpeed1;
    public double current2,currentAlert2,Ticks2,RPM2,AngularSpeed2,LiniarSpeed2;
    double anglePoz;
    double distance1,distance2;
    private TreeMap<Double, Double> AnglerPozToHoodAngle = new TreeMap<>();
    DetectedColor color1,color2;
    public enum DetectedColor {
        GREEN,
        PURPLE,
        UNKNOWN
    }
    public enum JumpState{
        IDLE,
        JUMPING
    }
    public enum MotorState{
        OFF,
        STANDBY,
        READY,
        SPEEDING_UP,
        SLOWING_DOWN

    }
    public enum RequestedShoot{
        PURPLE,
        GREEN,
        NONE
    }
    public enum ShootState{
        IDLE,
        SHOT_GREEN,
        SHOT_PURPLE,
        UNAVAILABLE,
        WAIT

    }
    public enum Patterns{
        GPP,
        PGP,
        PPG,
        IDLE
    }
    public enum PatternState{
        IDLE,
        SHOOT_GREEN,
        SHOT_GREEN,
        SHOOT_PURPLE1,
        SHOT_PURPLE1,
        SHOOT_PURPLE2,
        SHOT_PURPLE2
    }
    DistanceSensor distanceSensor1;
    DistanceSensor distanceSensor2;
    public double SHOT_DELAY = 2.5;
    public double SHOT_DELAY2 = 2.5;
    public Patterns Pattern = Patterns.IDLE;
    public PatternState patternState = PatternState.IDLE;
    public RequestedShoot Request;
    public boolean IsAuto = true;
    public ShootState Shoot1State,Shoot2State;
    public MotorState MotorsState = MotorState.OFF;
    public ElapsedTime MotorsTimer;
    public JumpState Jump1State = JumpState.IDLE;
    public JumpState Jump2State = JumpState.IDLE;
    private ElapsedTime Jump1Timer,Jump2Timer;
    private ElapsedTime Shoot1Timer,Shoot2Timer;
    public ElapsedTime PatternTime,ShotCePrindeTime;
    static double ANG_departe = 0.17;
    static double ANG_aproape = 0.85;
    public static double TargetRPM = 2900;
    double RPM;
    public enum OverrideShootState {
        IDLE,
        JUMP1_1, WAIT1, FEED1,
        JUMP2_1, WAIT2, FEED2,
        JUMP1_2, WAIT3, FEED3,
        JUMP2_2, DONE
    }


    public OverrideShootState overrideShootState = OverrideShootState.IDLE;
    ElapsedTime overrideTimer;

    public Outtake (HardwareMap hw){
        Pattern = Patterns.IDLE;
        Jump1Timer = new ElapsedTime();
        Jump2Timer = new ElapsedTime();
        MotorsTimer = new ElapsedTime();
        Shoot1Timer = new ElapsedTime();
        Shoot2Timer = new ElapsedTime();
        PatternTime = new ElapsedTime();
        ShotCePrindeTime = new ElapsedTime();
        overrideTimer = new ElapsedTime();

        motor_shooter_1 = hw.get(DcMotorEx.class, "MotorShooter_2");
        motor_shooter_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_shooter_1.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients1 = new PIDFCoefficients(P1,0,0,F1);
        motor_shooter_1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients1);

        motor_shooter_2 = hw.get(DcMotorEx.class, "MotorShooter_1");
        motor_shooter_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_shooter_2.setDirection(DcMotorSimple.Direction.FORWARD);
        PIDFCoefficients pidfCoefficients2 = new PIDFCoefficients(P2,0,0,F2);
        motor_shooter_2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients2);

        jumper1 = hw.servo.get("Jumper_1");
        jumper2 = hw.servo.get("Jumper_2");
        angler = hw.servo.get("Angler");

        volt = hw.voltageSensor.iterator().next();
        colorSensor1 = hw.get(NormalizedColorSensor.class,"SensorColor_1");
        distanceSensor1 = (DistanceSensor) colorSensor1;
        colorSensor2 = hw.get(NormalizedColorSensor.class, "SensorColor_2");
        distanceSensor2 = (DistanceSensor) colorSensor2;
        colorSensor1.setGain(2f);
        colorSensor2.setGain(2f);

        jumper1.setPosition(LOW1);
        jumper2.setPosition(LOW2);
        angler.setPosition(0);


        AnglerPozToHoodAngle.put(0.1,30.0); //ohoho cat avem de scris aici(dependenta intre unghiul hoodului si pozitia servoului)


    }
    private DetectedColor detectColor(
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
            return DetectedColor.GREEN;
        else if (hue > 205 && hue < 295)
            return DetectedColor.PURPLE;
        else return DetectedColor.UNKNOWN;
    }
    public DetectedColor getDetectedColor1(Telemetry t){
        return detectColor(colorSensor1, t, "Sensor 1");
    }
    public DetectedColor getDetectedColor2(Telemetry t){
        return detectColor(colorSensor2, t, "Sensor 2");
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
        distance1 = distanceSensor1.getDistance(DistanceUnit.CM);
        color2 = getDetectedColor2(t);
        distance2 = distanceSensor2.getDistance(DistanceUnit.CM);

        t.addData("Sensor 1", "%s", color1);
        t.addData("Sensor 2", "%s", color2);

        anglePoz = AnglePoz();
        t.addData("Shooter Poz","%f",anglePoz);
        t.addData("Jumper1","%f",jumper1.getPosition());

        t.addData("PatternState",patternState);
        t.addData("ShootState1",Shoot1State);
        t.addData("ShootState2",Shoot2State);
        t.addData("Pattern",Pattern);

        t.addData("Current Angler Pos:", angler.getPosition());
        RPM = (RPM1+RPM2)/2;
        t.addData("RPM mediu:", RPM);
        t.addData("Stop: ", EnteredStop);
        t.addData("request", Request);
        t.addData("MotorState:", MotorsState);

    }

    public void Update(){ //valorile cu secundele trebuie reglate
        if(Jump1State == JumpState.JUMPING && Jump1Timer.seconds()>0.3){
            Low1();
            Jump1State = JumpState.IDLE;
        }
        if(Jump2State == JumpState.JUMPING && Jump2Timer.seconds()>0.3){
            Low2();
            Jump2State = JumpState.IDLE;
        }
        if(MotorsState == MotorState.SPEEDING_UP && RPM >= TargetRPM){
            MotorsState = MotorState.READY;
        }

        if(Request == RequestedShoot.GREEN) {
            if (OneCanShootGreen() && TwoCanShootGreen() && Shoot1State == ShootState.WAIT && MotorsState == MotorState.READY) {
                Shoot1Timer.reset();
                StartJump1();
                Request = RequestedShoot.NONE;
                if (patternState == PatternState.IDLE) {
                    Shoot1State = ShootState.SHOT_GREEN;
                }
            }

            else if (OneCanShootGreen() && Shoot1State == ShootState.WAIT && MotorsState == MotorState.READY) {
                Shoot1Timer.reset();
                Request = RequestedShoot.NONE;
                StartJump1();
                if (patternState == PatternState.IDLE) {
                    Shoot1State = ShootState.SHOT_GREEN;
                }
              }
              else if (TwoCanShootGreen() && Shoot2State == ShootState.WAIT && MotorsState == MotorState.READY) {
                Shoot2Timer.reset();
                Request = RequestedShoot.NONE;
                StartJump2();
                if (patternState == PatternState.IDLE) {
                    Shoot2State = ShootState.SHOT_GREEN;
                }
             }
        }
        if(Request == RequestedShoot.PURPLE) {
             if (OneCanShootPurple() && TwoCanShootPurple() && Shoot1State == ShootState.WAIT && MotorsState == MotorState.READY) {
                Shoot1Timer.reset();
                StartJump1();
                Request = RequestedShoot.NONE;
                if (patternState == PatternState.IDLE) {
                    Shoot1State = ShootState.SHOT_PURPLE;
                }
             } else if (OneCanShootPurple() && Shoot1State == ShootState.WAIT && MotorsState == MotorState.READY) {
                Shoot1Timer.reset();
                StartJump1();
                Request = RequestedShoot.NONE;
                if (patternState == PatternState.IDLE) {
                    Shoot1State = ShootState.SHOT_PURPLE;
                }
             } else if (TwoCanShootPurple()&& Shoot2State == ShootState.WAIT && MotorsState == MotorState.READY) {
                Shoot2Timer.reset();
                StartJump2();
                Request = RequestedShoot.NONE;
                if (patternState == PatternState.IDLE) {
                    Shoot2State = ShootState.SHOT_PURPLE;
                }
               }
        }
        if(MotorsState == MotorState.READY && (Shoot1State == ShootState.IDLE || Shoot1State == ShootState.SHOT_GREEN || Shoot1State == ShootState.SHOT_PURPLE) && Shoot1Timer.seconds()>WAIT){
            Stop();
            MotorsState = MotorState.OFF;
            Shoot1State = ShootState.WAIT;
        }
        if(MotorsState == MotorState.READY && (Shoot2State == ShootState.IDLE || Shoot2State == ShootState.SHOT_GREEN || Shoot2State == ShootState.SHOT_PURPLE) && Shoot2Timer.seconds()>WAIT){
            Stop();
            MotorsState = MotorState.OFF;
            Shoot2State = ShootState.WAIT;
        }
        updateShootOverride();



        switch (Pattern) {
            case GPP:
                handleGPP();
                break;
            case PGP:
                handlePGP();
                break;
            case PPG:
                handlePPG();
                break;
        }
    }
    private void handlePPG(){
        switch (patternState){
            case IDLE:
                break;
            case SHOOT_PURPLE1:
                ShootPurple();
                PatternTime.reset();
                patternState = PatternState.SHOT_PURPLE1;
                break;
            case SHOT_PURPLE1:
                if(Request == RequestedShoot.NONE  && PatternTime.seconds()>1.5) {
                    IntakeTimer.reset();
                    Intake.intakeStates = Intake.IntakeStates.PUSHING;
                }
                if(PatternTime.seconds() > 4){
                    patternState = PatternState.SHOOT_PURPLE2;
                   // BAG_PULA = true;
                }

                break;
            case SHOOT_PURPLE2:
                ShootPurple();
                PatternTime.reset();
                patternState = PatternState.SHOT_PURPLE2;
                break;
            case SHOT_PURPLE2:
                if(PatternTime.seconds() > SHOT_DELAY && (Shoot1State == ShootState.WAIT || Shoot2State == ShootState.WAIT)){
                    patternState = PatternState.SHOOT_GREEN;
                    //BAG_PULA = false;
                }
                break;
            case SHOOT_GREEN:
                ShootGreen();
                PatternTime.reset();
                patternState = PatternState.SHOT_GREEN;
                break;
            case SHOT_GREEN:
                if(PatternTime.seconds() > SHOT_DELAY){
                    patternState = PatternState.IDLE;// pattern terminat
                    Shoot1State = ShootState.IDLE;
                    Shoot2State = ShootState.IDLE;
                }
        }
    }
    private void handlePGP(){
        switch (patternState){
            case IDLE:
                break;
            case SHOOT_PURPLE1:
                ShootPurple();
                PatternTime.reset();
                patternState = PatternState.SHOT_PURPLE1;
                break;
            case SHOT_PURPLE1:
                if(PatternTime.seconds() > SHOT_DELAY && (Shoot1State == ShootState.WAIT || Shoot2State == ShootState.WAIT)){
                    patternState = PatternState.SHOOT_GREEN;
                }
                break;
            case SHOOT_GREEN:
                IntakeTimer.reset();
                Intake.intakeStates = Intake.IntakeStates.PUSHING;
                ShootGreen();
                PatternTime.reset();
                patternState = PatternState.SHOT_GREEN;
                break;
            case SHOT_GREEN:
                if (PatternTime.seconds() > SHOT_DELAY) {
                    patternState = PatternState.SHOOT_PURPLE2;
                }
                break;
            case SHOOT_PURPLE2:
                ShootPurple();
                PatternTime.reset();
                patternState = PatternState.SHOT_PURPLE2;
                break;
            case SHOT_PURPLE2:
                if (PatternTime.seconds() > SHOT_DELAY2) {
                    patternState = PatternState.IDLE;// pattern terminat
                    Shoot1State = ShootState.IDLE;
                    Shoot2State = ShootState.IDLE;
                }
                break;
        }
    }
    private void handleGPP(){
        switch (patternState) {

            case IDLE:
                break;
            case SHOOT_GREEN:
                ShootGreen();
                IntakeTimer.reset();
                Intake.intakeStates = Intake.IntakeStates.PUSHING;
                break;

            case SHOT_GREEN:
                if (PatternTime.seconds() > SHOT_DELAY && (Shoot1State == ShootState.WAIT || Shoot2State == ShootState.WAIT)) {
                    patternState = PatternState.SHOOT_PURPLE1;
                }
                break;

            case SHOOT_PURPLE1:
                PatternTime.reset();        // resetezi la  nceputul pattern-ului
                patternState = PatternState.SHOT_GREEN;
                ShootPurple();
                PatternTime.reset();        // resetam timerul doar la START-ul SHOT_PURPLE1
                patternState = PatternState.SHOT_PURPLE1;
                break;

            case SHOT_PURPLE1:
                if (PatternTime.seconds() > SHOT_DELAY) {
                    patternState = PatternState.SHOOT_PURPLE2;
                }
                break;

            case SHOOT_PURPLE2:
                ShootPurple();
                PatternTime.reset();
                patternState = PatternState.SHOT_PURPLE2;
                break;

            case SHOT_PURPLE2:
                if (PatternTime.seconds() > SHOT_DELAY2) {
                    patternState = PatternState.IDLE;// pattern terminat
                    Shoot1State = ShootState.IDLE;
                    Shoot2State = ShootState.IDLE;
                }
                break;

        }

    }
    public void ShootPattern(){
        switch (Pattern){
            case GPP:
                patternState = PatternState.SHOOT_GREEN;
                break;
            case PGP:
                patternState = PatternState.SHOOT_PURPLE1;
                break;
            case PPG:
                patternState = PatternState.SHOOT_PURPLE1;
                break;
        }
    }
    int state = 1;
    public void ShootOverride() {
        AutoShoot();                 // pornește motoarele la TargetRPM
        overrideShootState = OverrideShootState.JUMP1_1;
        overrideTimer.reset();
    }
    public void updateShootOverride() {

        if (overrideShootState == OverrideShootState.IDLE) return;
        if (RPM<=TargetRPM) return;

        switch (overrideShootState) {

            // J1
            case JUMP1_1:
                Jump1();
                overrideTimer.reset();
                overrideShootState = OverrideShootState.WAIT1;
                break;

            case WAIT1:
                if (overrideTimer.seconds() > 0.3) {
                    Low1();
                    overrideTimer.reset();
                    overrideShootState = OverrideShootState.FEED1;
                }
                break;

            case FEED1:
                IntakeTimer.reset();
                Intake.intakeStates = Intake.IntakeStates.PUSHING;
                overrideTimer.reset();
                overrideShootState = OverrideShootState.JUMP2_1;
                break;

            // J2
            case JUMP2_1:
                if (overrideTimer.seconds() > 1.2) {
                    Jump2();
                    overrideTimer.reset();
                    overrideShootState = OverrideShootState.WAIT2;
                }
                break;

            case WAIT2:
                if (overrideTimer.seconds() > 0.3) {
                    Low2();
                    overrideTimer.reset();
                    overrideShootState = OverrideShootState.FEED2;
                }
                break;

            case FEED2:
                IntakeTimer.reset();
                Intake.intakeStates = Intake.IntakeStates.PUSHING;
                overrideTimer.reset();
                overrideShootState = OverrideShootState.JUMP1_2;
                break;

            // J1
            case JUMP1_2:
                if (overrideTimer.seconds() > 1.2) {
                    Jump1();
                    overrideTimer.reset();
                    overrideShootState = OverrideShootState.WAIT3;
                }
                break;

            case WAIT3:
                if (overrideTimer.seconds() > 0.3) {
                    Low1();
                    overrideTimer.reset();
                    overrideShootState = OverrideShootState.FEED3;
                }
                break;

            case FEED3:
                IntakeTimer.reset();
                Intake.intakeStates = Intake.IntakeStates.PUSHING;
                overrideTimer.reset();
                overrideShootState = OverrideShootState.JUMP2_2;
                break;

            // J2 final
            case JUMP2_2:
                if (overrideTimer.seconds() > 1.2) {
                    Jump2();
                    overrideTimer.reset();
                    overrideShootState = OverrideShootState.DONE;
                }
                break;

            case DONE:
                if (overrideTimer.seconds() > 0.3) {
                    Low2();
                    Stop();
                    MotorsState = MotorState.OFF;
                    overrideShootState = OverrideShootState.IDLE;
                }
                break;
        }
    }


    public void ShootGreen(){
        if(OneCanShootGreen()){
            Request = RequestedShoot.GREEN;
            AutoShoot();
            Shoot1Timer.reset();
            Shoot1State = ShootState.WAIT;
        }else if(TwoCanShootGreen()){
            Request = RequestedShoot.GREEN;
            AutoShoot();
            Shoot2Timer.reset();
            Shoot2State = ShootState.WAIT;
        }
    }
    public void ShootPurple(){
        if(OneCanShootPurple()){
            Request = RequestedShoot.PURPLE;
            AutoShoot();
            Shoot1Timer.reset();
            Shoot1State = ShootState.WAIT;
        }else if(TwoCanShootPurple()){
            Request = RequestedShoot.PURPLE;
            AutoShoot();
            Shoot2Timer.reset();
            Shoot2State = ShootState.WAIT;
        }
    }
    public void AutoShoot(){
        if(TargetRPM == 2500){
            ShootApr();
        }else if(TargetRPM == 2900){
            Shoot();
        }
        MotorsState = MotorState.SPEEDING_UP;
    }

    public void StartJump1(){
        if (Jump1State != JumpState.IDLE) return;
        Jump1();
        Jump1Timer.reset();
        Jump1State = JumpState.JUMPING;
    }
    public void StartJump2(){
        if (Jump2State != JumpState.IDLE) return;
        Jump2();
        Jump2Timer.reset();
        Jump2State = JumpState.JUMPING;
    }
    public void AngleApr(){
        angler.setPosition(ANG_aproape);
    }
    public void AngleDep(){
        angler.setPosition(ANG_departe);
    }

    public double AnglePoz(){
        return angler.getPosition();
    }

    public void Shoot() {
        motor_shooter_1.setVelocity(1630);
        motor_shooter_2.setVelocity(1630);
    }
    public void ShootApr(){
        motor_shooter_1.setVelocity(1370);
        motor_shooter_2.setVelocity(1370);
    }
    public boolean EnteredStop = false;
    public void Stop(){
        EnteredStop = true;
        motor_shooter_1.setVelocity(0);
        motor_shooter_2.setVelocity(0);
    }
    public void stop2(){
        motor_shooter_1.setVelocity(0);
        motor_shooter_2.setVelocity(0);
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
    public void MJump1(){
        jumper1.setPosition(LOW1+0.15);
    }
    public void MJump2(){
        jumper2.setPosition(LOW2+0.15);
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

    public double getJ1() {
        return jumper1.getPosition();
    }

    public double getJ2() {
        return jumper2.getPosition();
    }


    public boolean OneCanShootGreen(){
        if (color1 == DetectedColor.GREEN){
            return true;
        }else return false;
    }
    public boolean OneCanShootPurple(){
        if(color1 == DetectedColor.PURPLE){
            return true;
        }else return false;
    }
    public boolean TwoCanShootGreen(){
        if(color2 == DetectedColor.GREEN){
            return true;
        }else return false;
    }
    public boolean TwoCanShootPurple(){
        if(color2 == DetectedColor.PURPLE){
            return true;
        }else return false;
    }

    public void AnglerMoveForward(){
        angler.setPosition(angler.getPosition() + 0.05);
    }

    public void AnglerMoveBackward(){
        angler.setPosition(angler.getPosition() - 0.05);
    }
}