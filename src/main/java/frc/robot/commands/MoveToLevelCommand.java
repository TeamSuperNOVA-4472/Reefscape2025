package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.objectmodels.CarriagePreset;
import frc.robot.subsystems.ElevatorCarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * Moves the elevator to a specific preset.
 * THIS DOES NOT CHECK IF THE CARRIAGE IS IN "SAFE" MODE!!
 * Invoke `moveToLevelSafe` to stow the carriage before moving.
 */
public class MoveToLevelCommand extends Command
{
    private final ElevatorCarriageSubsystem mElevatorCarriageSubsystem;
    private final CarriagePreset mPreset;

    public MoveToLevelCommand(CarriagePreset pPreset)
    {
        mElevatorCarriageSubsystem = ElevatorCarriageSubsystem.instance();
        mPreset = pPreset;
        addRequirements(mElevatorCarriageSubsystem);
    }

    @Override
    public void initialize() 
    {
        mElevatorCarriageSubsystem.setPreset(mPreset);
    }
    @Override
    public boolean isFinished()
    {
        if (mElevatorCarriageSubsystem.getActivePreset().isEmpty())
        {
            return true;
        }

        return Math.abs(mElevatorCarriageSubsystem.getElevatorHeight() - mElevatorCarriageSubsystem.getActivePreset().get().kElevatorPreset) < 1.2;
    }
}
