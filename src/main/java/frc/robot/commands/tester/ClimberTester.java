package frc.robot.commands.tester;

import java.util.function.Supplier;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimberTester extends Command 
{
    private final Supplier<Boolean> mClimbIn;
    private final Supplier<Boolean> mClimbOut;
    private final Supplier<Double> mArmsIn;
    private final ClimbSubsystem mClimb;
    
    public ClimberTester(ClimbSubsystem pClimb, Supplier<Boolean> pClimbIn, Supplier<Boolean> pClimbOut, Supplier<Double> pArmsIn)
    {
       mClimbIn = pClimbIn;
       mClimbOut = pClimbOut;
       
       mArmsIn = pArmsIn;

       mClimb = pClimb;
       addRequirements(pClimb);
    }    

    @Override
    public void execute() 
    {
        if(mClimbIn.get())
        {
            mClimb.setClimbVoltage(4);
        } 
        else if(mClimbOut.get())
        {
            // FIXME: Move all this to ClimbTeleop!!
            mClimb.setClimbVoltage(-4);
        } 
        else
        {
            mClimb.setClimbVoltage(0);
        }

    }
}
