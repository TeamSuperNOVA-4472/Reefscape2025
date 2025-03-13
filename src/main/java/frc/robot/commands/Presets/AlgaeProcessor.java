package frc.robot.commands.Presets;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveCarriageToPresetCommand;
import frc.robot.commands.moveToLevelSafe;
import frc.robot.objectmodels.CarriagePreset;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Moves the carriage and the elevator to the processor preset. Does not begin outtaking algae.
 */
public class AlgaeProcessor extends SequentialCommandGroup
{
    public AlgaeProcessor()
    {
        addCommands(
            new moveToLevelSafe(CarriagePreset.kAlgaeProcessor),
            new MoveCarriageToPresetCommand(CarriagePreset.kAlgaeProcessor)
        );
    }    
}
