package frc.robot.commands.Presets;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveCarriageToPresetCommand;
import frc.robot.commands.moveToLevelSafe;
import frc.robot.objectmodels.CarriagePreset;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Puts the carriage in its "default" position, and puts the elevator on the ground.
 * If the algae is detected, it will go to a different position to accomodate.
 */
public class StowCarriagePosition extends SequentialCommandGroup 
{
    public StowCarriagePosition()
    {
        IntakeSubsystem intake = IntakeSubsystem.instance();
        CarriagePreset stowPreset;
        if (intake.hasAlgae()) stowPreset = CarriagePreset.kStowAlgae;
        else stowPreset = CarriagePreset.kStowCoral;

        addCommands(
            new moveToLevelSafe(stowPreset),
            new MoveCarriageToPresetCommand(stowPreset),
            new InstantCommand(
                () -> CarriageSubsystem.instance().setAlgaeMode(intake.hasAlgae())
            )
        );
    }    
}
