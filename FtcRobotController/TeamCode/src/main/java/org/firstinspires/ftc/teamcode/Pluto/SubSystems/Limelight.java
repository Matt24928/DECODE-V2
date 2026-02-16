package org.firstinspires.ftc.teamcode.Pluto.SubSystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class Limelight extends SubsystemBase {
    private Limelight3A limelight;
    private LLResult llResult;

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
    public Pose3D GetDistance(){
        return llResult.getBotpose();
    }

    public double getTyDeg() {
        return hasTarget() ? llResult.getTy() : 0.0;
    }
    public double getTa(){
        return hasTarget() ? llResult.getTa() : 0.0;
    }

}
