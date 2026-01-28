package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Sorter extends SubsystemBase {
    Servo sorter;
    double left=0.7, right=0.37,zero=0.55;
    public Sorter(HardwareMap hw){
        sorter = hw.servo.get("Sorter");
    }
    public void moveP(){
        sorter.setPosition(sorter.getPosition()+0.01);
    }
    public void moveB(){
        sorter.setPosition(sorter.getPosition()-0.01);
    }

    public double getPos(){
        return sorter.getPosition();
    }

    public void Left(){
        sorter.setPosition(left);
    }
    public void Right(){
        sorter.setPosition(right);
    }

    public void Middle(){
        sorter.setPosition(zero);
    }

}
