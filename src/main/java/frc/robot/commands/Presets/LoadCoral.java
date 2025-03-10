package frc.robot.commands.Presets;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveCarriageToPresetCommand;
import frc.robot.commands.MoveToLevelCommand;
import frc.robot.commands.moveToLevelSafe;
import frc.robot.objectmodels.CarriagePreset;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Moves the carriage to a position to load coral. Does not begin intaking.
 */
public class LoadCoral extends SequentialCommandGroup
{
    public LoadCoral(ElevatorSubsystem pElevator, CarriageSubsystem pCarriage, IntakeSubsystem pIntake)
    {
        // I didn't feel fully comfortable getting rid of the old code, but
        // this should in theory do the same thing. It appears this code was
        // written before the `moveToLevelSafe` command.
        
        /*addCommands(
            new MoveCarriageToPresetCommand(pCarriage, CarriageSubsystem.armMovingAngle, CarriageSubsystem.wristMovingAngle),
            new MoveToLevelCommand(pElevator, ElevatorSubsystem.initialHeight),
            new MoveCarriageToPresetCommand(pCarriage, CarriageSubsystem.armCoralLoad, CarriageSubsystem.wristCoralLoad),
            new MoveToLevelCommand(pElevator, ElevatorSubsystem.kPresetCoralPickup),
            new InstantCommand(() -> pCarriage.setAlgaeMode(false)) // TODO: Would this cause a problem?
        );*/

        addCommands(
            new moveToLevelSafe(pCarriage, pElevator, pIntake, CarriagePreset.kCoralLoad),
            new MoveCarriageToPresetCommand(pCarriage, CarriagePreset.kCoralLoad)
        );
    }
}