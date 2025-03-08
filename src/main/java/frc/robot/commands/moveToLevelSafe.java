package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.objectmodels.CarriagePreset;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Moves both the elevator and carriage to a new preset.
 * This first moves the carriage to a "safe" stow position, then moves the elevator,
 * and then puts the carriage where it needs to be.
 */
public class moveToLevelSafe extends SequentialCommandGroup
{
    public moveToLevelSafe(CarriagePreset pNewLevel) 
    {
        addCommands(
            new moveOutOfDangerZone(),
            new MoveToLevelCommand(pNewLevel)
        );
    }
}
