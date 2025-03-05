package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.objectmodels.CarriagePreset;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * Moves the elevator to a specific preset.
 * THIS DOES NOT CHECK IF THE CARRIAGE IS IN "SAFE" MODE!!
 * Invoke `moveToLevelSafe` to stow the carriage before moving.
 */
public class MoveToLevelCommand extends Command
{
    private final ElevatorSubsystem mElevatorSubsystem;
    private final CarriagePreset mPreset;

    public MoveToLevelCommand(ElevatorSubsystem pElevatorSubsystem, CarriagePreset pPreset)
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
