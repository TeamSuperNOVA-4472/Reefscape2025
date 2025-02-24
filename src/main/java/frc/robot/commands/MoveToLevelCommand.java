package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class MoveToLevelCommand extends Command
{
    private final ElevatorSubsystem mElevatorSubsystem;
    private final Double mPreset;

    public MoveToLevelCommand(ElevatorSubsystem pElevatorSubsystem, Double pPreset)
    {
            mElevatorSubsystem = pElevatorSubsystem;
            mPreset = pPreset;
            addRequirements(mElevatorSubsystem);
    }

    @Override
    public void initialize() 
    {
        mElevatorSubsystem.setPreset(mPreset);
    }
    @Override
    public boolean isFinished()
    {
        return Math.abs(mElevatorSubsystem.getElevatorHeight() - mElevatorSubsystem.getElevatorPreset()) < 1.2;
    }
}
