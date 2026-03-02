package org.firstinspires.ftc.teamcode.Pluto.SubSystems;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

public class Limelight extends SubsystemBase {
    public Limelight3A limelight;
    private LLResult llResult;
    private PathChain getArtefact;
    public double DX, DY, SampleX, SampleY, ConstFwd = 0.03, ConstStr = 0.03, Xtarget, Ytarget;
    double heading;

    public boolean prea_mare, prea_mare_too;
    public double CamOffsetFromCenter = 4.1338582677;

    public Limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");// AprilTag pipeline
        limelight.setPollRateHz(90);
        limelight.start();
    }

    @Override
    public void periodic() {
        llResult = limelight.getLatestResult();
    }

    /* ===== BASIC DATA ===== */
    public Position GetPoz(){
        return llResult.getBotpose().getPosition();
    }
    public boolean hasTarget() {
        return llResult != null && llResult.isValid();
    }
    public Pose3D GetDistance(){
        return llResult.getBotpose();
    }

    public double getTyDeg() {
        return hasTarget() ? llResult.getTy() : 0.0;
    }
    public double getTa(){
        return hasTarget() ? llResult.getTa() : 0.0;
    }

    public void getResults() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
            for (LLResultTypes.DetectorResult detection : detections) {
//                String className = detection.getClassName();
                double x = detection.getTargetXDegrees();
                double y = detection.getTargetYDegrees();

                DY = Math.tan(Math.toRadians(y + 75)) * 15.566929134;
                DX = Math.tan(Math.toRadians(x)) * DY;
 //               if(y>14) prea_mare = true;
 //               else prea_mare = false;

//                if(y<1) prea_mare_too = true;
//                else prea_mare_too = false;
                // telemetry.addData(className + " #" + i, "at (" + x + ", " + y + ") degrees");
                // telemetry.addLine("GOOD TARGET");
//                DX = Math.tan(Math.toRadians(x + 90)) * 14.566929134;//DX e distanta fata
//                if(DX>75){
//                    DX=74;
//                }
//                if(DX>75){
//                    DX=71;
 //               }
 //               DY = Math.tan(Math.toRadians(y)) * DX -1;//distanta laterala
                break;
            }
        }
    }

    public void Rich_Artefact(Follower follower){
        // obiect stanga, ty cu plus, obiect mai sus de centru, tx cu plus
        double robotX = follower.getPose().getX(); // cum merge el in fata
        double robotY = follower.getPose().getY(); // cum merge el in laterala
        double headingRad = follower.getPose().getHeading();

        double xField, yField;

        if(DY<0){
            yField = robotY + Math.abs(DY) + CamOffsetFromCenter;
        }
        else{
            yField = robotY -DY + CamOffsetFromCenter;
        }

        xField = robotX;

//        if(DX > 0){
//            dxCam = DX - Math.abs(CamOffsetFromCenter);
//        }else {
//            //if(Math.abs(DX) < Math.abs(camOffsetSideways) dxCam = DX - Math.abs(camOffsetSideways);
//            dxCam = DX - Math.abs(CamOffsetFromCenter);
//        }

//        double dxField = -Math.sin(headingRad) * dxRobot + Math.cos(headingRad) * dyRobot; // înainte
//        double dyField =  Math.cos(headingRad) * dxRobot + Math.sin(headingRad) * dyRobot; // stânga
//        ArtefactPose = new Pose(xField,yField);

    }



}
