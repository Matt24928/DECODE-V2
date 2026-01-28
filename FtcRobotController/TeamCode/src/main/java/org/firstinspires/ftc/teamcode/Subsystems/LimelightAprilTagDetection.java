package org.firstinspires.ftc.teamcode.Subsystems;


import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.control.PIDFController;
import com.pedropathing.control.PIDFCoefficients;


import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
@Configurable
public class LimelightAprilTagDetection extends OpMode {

    Limelight3A limelight;
    private Follower follower;
//    private double distance;
//    private double currentHeading;
//    private double targetHeading;
    private double rotation = 0;
    static double kp=1.2,kd=0.05,kf;
    private boolean automaticHeading = false;
    private final Pose startingPose = new Pose(56, 8, Math.toRadians(90));
    public static PIDFCoefficients pidfCoefficients = new PIDFCoefficients(kp,0,kd,kf);
    static PIDFController headingController;


    @Override
    public void init(){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(3);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.startTeleopDrive(false);
        headingController = new PIDFController(pidfCoefficients);
    }

    @Override
    public void start(){
        limelight.start();
    }


    @Override
    public void loop(){
        follower.update();
        headingController.setCoefficients(pidfCoefficients);
        LLResult llResult = limelight.getLatestResult();
        if(llResult!= null && llResult.isValid() && automaticHeading == true){
            double txDeg = llResult.getTx();

            // deadzone ca să nu tremure
            double headingErrRad;
            if (Math.abs(txDeg) < 1.0) {
                headingErrRad = 0;
            } else {
                headingErrRad = -Math.toRadians(txDeg);
            }
            headingController.updateError(headingErrRad);
            rotation = headingController.run();
            if (Math.abs(rotation) < 0.05) {
                rotation = 0;
            }
//            Pose3D botpose = llResult.getBotpose_MT2();
//            double tx = Math.toRadians(llResult.getTx());
//            currentHeading = follower.getPose().getHeading();
//            targetHeading = currentHeading + tx;
            //follower.setHeading(targetHeading);
            telemetry.addData("Target Area: ", llResult.getTa());
            telemetry.addData("Target X: ", llResult.getTx());
            telemetry.addLine("I SEE IT");
           // telemetry.addData("BotPose: ", botpose.toString());
        }
        else{
            telemetry.addLine("NO TARGET");
        }
        if(automaticHeading == false) {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true
            );
        }
        else if(llResult!=null){
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    rotation,
                    true
            );
        }else {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true
            );
        }
        if(gamepad1.xWasPressed()){
            automaticHeading = true;
            headingController.reset();
        }
        if(gamepad1.yWasPressed()){
            automaticHeading = false;
            headingController.reset();
        }
        telemetry.addData("kp:",kp);
        telemetry.addData("kd:",kd);

        telemetry.update();
    }

//    public void getTargetDistance(double ta){
//
//    }
}
