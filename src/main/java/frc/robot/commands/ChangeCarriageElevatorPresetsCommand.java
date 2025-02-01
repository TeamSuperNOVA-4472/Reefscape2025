package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.objectmodels.IntakePresets;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

// Executes a series of commands in order.
// Given a new preset to move to, the command:
// - Retracts the carriage.
// - Moves the elevator to its desired location.
// - Places the carriage arm & wrist where they belong.
public class ChangeCarriageElevatorPresetsCommand extends SequentialCommandGroup
{
    public ChangeCarriageElevatorPresetsCommand(CarriageSubsystem carriage,
                                                ElevatorSubsystem elevator,
                                                IntakePresets newPreset)
    {
        addRequirements(carriage, elevator);
        addCommands(
            new MoveCarriageToPresetCommand(carriage, IntakePresets.kAway),
            // new MoveToLevelCommand(elevator, newPreset),
            new MoveCarriageToPresetCommand(carriage, newPreset)
        );
    }
}
