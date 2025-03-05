package frc.robot.commands.Presets;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveCarriageToPresetCommand;
import frc.robot.commands.moveToLevelSafe;
import frc.robot.objectmodels.CarriagePreset;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Safely moves the carriage and the elevator to the barge preset to drop off algae.
 */
public class AlgaeBarge extends SequentialCommandGroup
{
    public AlgaeBarge(ElevatorSubsystem pElevator, CarriageSubsystem pCarriage, IntakeSubsystem pIntakeSubsystem)
    {
        addCommands(
            new moveToLevelSafe(pCarriage, pElevator, pIntakeSubsystem, CarriagePreset.kAlgaeBarge),
            new MoveCarriageToPresetCommand(pCarriage, CarriagePreset.kAlgaeBarge)
        );
    }    
}
