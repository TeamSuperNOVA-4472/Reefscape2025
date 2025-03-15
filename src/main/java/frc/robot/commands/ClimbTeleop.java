package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbTeleop extends Command 
{
    private static final double CLIMB_VOLTAGE = 12;

    private final ClimbSubsystem mClimbSubsystem;
    Supplier<Boolean> mClimbFwd;
    Supplier<Boolean> mClimbRev;

    public ClimbTeleop(Supplier<Boolean> pClimbFwd, Supplier<Boolean> pClimbRev)
    {
        mClimbSubsystem = ClimbSubsystem.kInstance;
        mClimbFwd = pClimbFwd;
        mClimbRev = pClimbRev;
        addRequirements(mClimbSubsystem);
    }    

    @Override
    public void execute()
    {
        if(mClimbFwd.get())
        {
            mClimbSubsystem.setVoltage(CLIMB_VOLTAGE);
        }
        else if(mClimbRev.get())
        {
            mClimbSubsystem.setVoltage(-CLIMB_VOLTAGE);
        }
        else
        {
            mClimbSubsystem.setVoltage(0);
        }
    }
}
