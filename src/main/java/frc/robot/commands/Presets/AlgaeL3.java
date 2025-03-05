package frc.robot.commands.Presets;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveCarriageToPresetCommand;
import frc.robot.commands.moveToLevelSafe;
import frc.robot.objectmodels.CarriagePreset;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Moves the robot to the L3 Algae position. Does not begin intaking. See `AlgaeL3IntakePreset` for that.
 */
public class AlgaeL3 extends SequentialCommandGroup
{
    public AlgaeL3(ElevatorSubsystem pElevator, CarriageSubsystem pCarriage, IntakeSubsystem pIntakeSubsystem)
    {
        addCommands(
            new moveToLevelSafe(pCarriage, pElevator, pIntakeSubsystem, CarriagePreset.kAlgaeL3),
            new MoveCarriageToPresetCommand(pCarriage, CarriagePreset.kAlgaeL3)
        );
    }
}
