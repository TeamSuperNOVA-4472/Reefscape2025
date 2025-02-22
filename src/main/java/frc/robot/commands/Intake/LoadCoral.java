package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.objectmodels.IntakePreset;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class LoadCoral extends SequentialCommandGroup
{
    public LoadCoral(ElevatorSubsystem pElevator, CarriageSubsystem pCarriage)
    {
        addCommands(
            new StowCarriagePosition(pCarriage, pElevator),
            new MoveCarriageToPresetCommand(pCarriage, IntakePreset.kLoadCoralStage1),
            new MoveCarriageToPresetCommand(pCarriage, IntakePreset.kLoadCoralStage2),
            new MoveToLevelCommand(pElevator, IntakePreset.kLoadCoralStage2)
        );
    }
}