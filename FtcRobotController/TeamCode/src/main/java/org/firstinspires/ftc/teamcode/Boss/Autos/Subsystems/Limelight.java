package org.firstinspires.ftc.teamcode.Boss.Autos.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class Limelight extends SubsystemBase {

    private final Limelight3A limelight;
    private LLResult llResult;
    double DX, DY, SampleX, SampleY, ConstFwd = 0.03, ConstStr = 0.03, Xtarget, Ytarget;
    double heading;

    private double CamOffsetFromCenter = 10;

    public Limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(3); // AprilTag pipeline
        limelight.setPollRateHz(100);
        limelight.start();
    }

    @Override
    public void periodic() {
        llResult = limelight.getLatestResult();
    }

    /* ===== BASIC DATA ===== */

    public boolean hasTarget() {
        return llResult != null && llResult.isValid();
    }

    public double getTxDeg() {
        return hasTarget() ? llResult.getTx() : 0.0;
    }

    public double getTa() {
        return hasTarget() ? llResult.getTa() : 0.0;
    }

    public void getResults() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
            for (LLResultTypes.DetectorResult detection : detections) {
                String className = detection.getClassName();
                double x = detection.getTargetXDegrees();
                double y = detection.getTargetYDegrees();
                // telemetry.addData(className + " #" + i, "at (" + x + ", " + y + ") degrees");
                // telemetry.addLine("GOOD TARGET");
                DY = Math.tan(Math.toRadians(y + 70)) * 9.7865;
                DX = Math.tan(Math.toRadians(x)) * DY;
                limelight.stop();
                break;
            }
        }
    }

}




