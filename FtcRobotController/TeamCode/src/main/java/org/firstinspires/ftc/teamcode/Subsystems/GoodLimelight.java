package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.List;
import java.util.regex.Pattern;

public class GoodLimelight{
    Limelight3A limelight;
    private LLResult result;
    public GoodLimelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(3);   // pipeline APRIL TAG
        limelight.setPollRateHz(100);
        limelight.start();
    }
    public void update() {
        LLResult latest = limelight.getLatestResult();
        if (latest != null && latest.isValid()) {
            result = latest;
        }
    }



    public Outtake.Patterns GetPattern() {
        if (result == null) {
            return Outtake.Patterns.IDLE;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            return Outtake.Patterns.IDLE;
        }

        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int id = fiducial.getFiducialId();
            switch (id) {
                case 21: return Outtake.Patterns.GPP;
                case 22: return Outtake.Patterns.PGP;
                case 23: return Outtake.Patterns.PPG;
            }
        }

        return Outtake.Patterns.IDLE;
    }

}
