package frc.robot.commands.Presets;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveCarriageToPresetCommand;
import frc.robot.commands.moveToLevelSafe;
import frc.robot.objectmodels.CarriagePreset;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Moves the carriage to a position to score Coral L4. Does not begin outtaking.
 */
public class CoralL4Preset extends SequentialCommandGroup
{
    public CoralL4Preset()
    {
        addCommands(
            new moveToLevelSafe(CarriagePreset.kCoralL4),
            new MoveCarriageToPresetCommand(CarriagePreset.kCoralL4)
        );
    }    
}
