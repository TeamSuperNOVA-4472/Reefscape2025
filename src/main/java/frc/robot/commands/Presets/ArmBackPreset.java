package frc.robot.commands.Presets;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveCarriageToPresetCommand;
import frc.robot.commands.moveToLevelSafe;
import frc.robot.objectmodels.CarriagePreset;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Moves the arm to the algae stow position, then to 95 degrees.
 * FIXME: This appears to be the result of a quick-fix for the climb. Please replace and remove.
 */
public class ArmBackPreset extends SequentialCommandGroup
{
    public ArmBackPreset()
    {
        addCommands(
            new moveToLevelSafe(CarriagePreset.kClimb),
            new MoveCarriageToPresetCommand(new CarriagePreset(CarriagePreset.kStowAlgae.kArmPreset, 10.0, 12.875)),
            new MoveCarriageToPresetCommand(new CarriagePreset(95.0, 10.0, 12.875))
        );
    }    
}
