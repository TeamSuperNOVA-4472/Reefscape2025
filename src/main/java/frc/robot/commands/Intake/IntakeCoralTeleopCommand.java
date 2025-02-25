package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.objectmodels.IntakePreset;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCoralTeleopCommand extends SequentialCommandGroup{

    public IntakeCoralTeleopCommand(
        CarriageSubsystem pCarriageSubsystem,
        ElevatorSubsystem pElevatorSubsystem,
        IntakeSubsystem pIntakeSubsystem,
        IntakePreset pPreset
    )
    {
        addCommands(
            new MoveToLevelSafe(pCarriageSubsystem, pElevatorSubsystem, pIntakeSubsystem, pPreset),
            new MoveCarriageToPresetCommand(pCarriageSubsystem, pPreset)
        );
    }
}
