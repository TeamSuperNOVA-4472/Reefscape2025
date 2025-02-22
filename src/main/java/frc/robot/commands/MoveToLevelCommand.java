package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.objectmodels.IntakePreset;
import frc.robot.subsystems.ElevatorSubsystem;

public class MoveToLevelCommand extends Command
{
    private final ElevatorSubsystem mElevatorSubsystem;
    private final IntakePreset mPreset;

    public MoveToLevelCommand(ElevatorSubsystem pElevatorSubsystem, IntakePreset pPreset)
    {
            mElevatorSubsystem = pElevatorSubsystem;
            mPreset = pPreset;
            addRequirements(mElevatorSubsystem);
    }

    @Override
    public void initialize() 
    {
        mElevatorSubsystem.setPreset(mPreset.elevatorPreset);
    }
    @Override
    public boolean isFinished()
    {
        return Math.abs(mElevatorSubsystem.getElevatorHeight() - mElevatorSubsystem.getElevatorPreset()) < 1.2;
    }
}
