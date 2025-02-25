package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.MoveCarriageToPresetCommand;
import frc.robot.commands.Intake.MoveToLevelCommand;
import frc.robot.objectmodels.IntakePreset;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class LoadCoral extends SequentialCommandGroup
{
    public LoadCoral(ElevatorSubsystem pElevator, CarriageSubsystem pCarriage, IntakeSubsystem pIntakeSubsystem)
    {
        addCommands(
            new MoveCarriageToPresetCommand(pCarriage, IntakePreset.kMoving),
            new MoveToLevelCommand(pElevator, IntakePreset.kMoving),
            new MoveCarriageToPresetCommand(pCarriage, IntakePreset.kLoadCoralStage1),
            new MoveCarriageToPresetCommand(pCarriage, IntakePreset.kLoadCoralStage2),
            new MoveToLevelCommand(pElevator, IntakePreset.kLoadCoralStage2),
            new InstantCommand(() -> pCarriage.setAlgaeMode(false))
        );
    }
}