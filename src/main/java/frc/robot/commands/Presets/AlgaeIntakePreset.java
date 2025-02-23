package frc.robot.commands.Presets;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveCarriageToPresetCommand;
import frc.robot.commands.MoveToLevelCommand;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class AlgaeIntakePreset extends SequentialCommandGroup
{
    public AlgaeIntakePreset(CarriageSubsystem pCarriage, ElevatorSubsystem pElevator)
    {
        addCommands(
            new AlgaeL2(pElevator, pCarriage),
            new InstantCommand(
                () -> pCarriage.setAlgaeMode(true)
            )
        );
    }    
}
