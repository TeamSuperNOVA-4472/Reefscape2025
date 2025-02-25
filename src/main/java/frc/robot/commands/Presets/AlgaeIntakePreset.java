package frc.robot.commands.Presets;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveCarriageToPresetCommand;
import frc.robot.commands.MoveToLevelCommand;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AlgaeIntakePreset extends SequentialCommandGroup
{
    public AlgaeIntakePreset(CarriageSubsystem pCarriage, ElevatorSubsystem pElevator, IntakeSubsystem pIntake)
    {
        addCommands(
            new AlgaeL2(pElevator, pCarriage, pIntake),
            new InstantCommand(
                () -> pCarriage.setAlgaeMode(true)
            )
        );
    }    
}
