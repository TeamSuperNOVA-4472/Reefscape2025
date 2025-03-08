package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbTeleop extends Command 
{
    private static final double GRAB_VOLTAGE = 0;

    private final ClimbSubsystem mClimbSubsystem;
    Supplier<Boolean> mGrabOnCageButton;
    Supplier<Boolean> mClimbUpButton;

    public ClimbTeleop(Supplier<Boolean> pGrabOnCageButton, Supplier<Boolean> pClimbUpButton)
    {
        mClimbSubsystem = ClimbSubsystem.instance();
        mClimbUpButton = pClimbUpButton;
        mGrabOnCageButton = pGrabOnCageButton;
    }    

    @Override
    public void execute()
    {
        if(mClimbUpButton.get())
        {
            mClimbSubsystem.closeClimber();
            mClimbSubsystem.setGrabberVoltage(0.0);
        }
        else if(mGrabOnCageButton.get())
        {
            mClimbSubsystem.openClimber();
            mClimbSubsystem.setGrabberVoltage(GRAB_VOLTAGE);
        }
        else
        {
            mClimbSubsystem.setGrabberVoltage(0);
        }
    }
}
