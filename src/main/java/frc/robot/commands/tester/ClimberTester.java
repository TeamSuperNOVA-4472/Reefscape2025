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
            mClimb.setLeftClimbVoltage(1);
            mClimb.setRightClimbVoltage(1);
            mClimb.setGrabberVoltage(1);
        } 
        else if(mClimbOut.get())
        {
            mClimb.setRightClimbVoltage(-3);
            mClimb.setLeftClimbVoltage(-3);
            mClimb.setGrabberVoltage(0);

        } 
        else
        {
            mClimb.setGrabberVoltage(0);
            mClimb.setRightClimbVoltage(0);
            mClimb.setLeftClimbVoltage(0);
        }

    }
}
