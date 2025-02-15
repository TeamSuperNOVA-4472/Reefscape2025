package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.objectmodels.IntakePresets;
import frc.robot.subsystems.ElevatorSubsystem;

public class MoveToLevelCommand extends Command
{
    private final ElevatorSubsystem mElevatorSubsystem;
    private final IntakePresets mPreset;

    public MoveToLevelCommand(ElevatorSubsystem pElevatorSubsystem, IntakePresets pPreset)
    {
        mElevatorSubsystem = pElevatorSubsystem;
        mPreset = pPreset;
    }

    @Override
    public void initialize() 
    {
        mElevatorSubsystem.setPreset(mPreset);
    }
    @Override
    public boolean isFinished()
    {
        return !mElevatorSubsystem.isMoving();
    }
}
