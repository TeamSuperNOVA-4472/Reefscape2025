package frc.robot.commands.Presets;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveCarriageToPresetCommand;
import frc.robot.commands.moveToLevelSafe;
import frc.robot.objectmodels.CarriagePreset;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Safely moves the carriage and the elevator to ground pickup for algae.
 */
public class AlgaeGround extends SequentialCommandGroup
{
    public AlgaeGround()
    {
        addCommands(
            new moveToLevelSafe(CarriagePreset.kAlgaeGround),
            new MoveCarriageToPresetCommand(CarriagePreset.kAlgaeGround),
            new MoveCarriageToPresetCommand(CarriagePreset.kAlgaeGround)
        );
    }    
}

