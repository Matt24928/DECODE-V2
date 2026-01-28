package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class Limelight extends SubsystemBase {

    private final Limelight3A limelight;
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

    public double getTxDeg() {
        return hasTarget() ? llResult.getTx() : 0.0;
    }
    public double getTa(){
        return hasTarget() ? llResult.getTa() : 0.0;
    }

}
