package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TimerCommand extends CommandBase {

    private ElapsedTime timer = new ElapsedTime();
    private double time;
    public TimerCommand(double time)
    {
        this.time = time;
    }

    @Override
    public void initialize()
    {
        timer.reset();
    }
    @Override
    public boolean isFinished()
    {
        return timer.milliseconds() >= time;
    }
}