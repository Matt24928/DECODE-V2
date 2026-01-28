package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.List;

public class Limelight extends SubsystemBase {
    private LLResult lastResult;
    public Limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(3);   // pipeline APRIL TAG
        limelight.setPollRateHz(50);
        limelight.start();
    }

    @Override
    public void periodic() {
         lastResult = limelight.getLatestResult();
    }
    public enum PatternState{
        ID21,
        ID22,
        ID23,
        NONE
    }

    public PatternState GetpatternState()
    {
        List<LLResultTypes.FiducialResult> ficu = lastResult.getFiducialResults();
        for(int i=0;i<ficu.size(); i++) {
            switch (ficu.get(i).getFiducialId()) {
                case 21:
                    return PatternState.ID21;
                case 22:
                    return PatternState.ID22;
                case 23:
                    return PatternState.ID23;
                default:
                    return PatternState.NONE;
            }
        }
        return null;
    }


    private final Limelight3A limelight;
    private LLResult result;

}







//Task de la sefu’: Sa facem o functie care retureza starea patternului si care va fi folosita in LL.
//Adica daca e id ul 21 22 23 . sa dea idul (tre sa fie independenta)