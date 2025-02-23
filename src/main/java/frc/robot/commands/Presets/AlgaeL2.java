package frc.robot.commands.Presets;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveCarriageToPresetCommand;
import frc.robot.commands.MoveToLevelCommand;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class AlgaeL2 extends SequentialCommandGroup{
    public AlgaeL2(ElevatorSubsystem pElevator, CarriageSubsystem pCarriage)
    {
        addCommands(
            new StowCarriagePositionAlgae(pCarriage, pElevator),
            new MoveToLevelCommand(pElevator, ElevatorSubsystem.kPresetAlgaeL2),
            new MoveCarriageToPresetCommand(pCarriage, CarriageSubsystem.armAlgaeL2, CarriageSubsystem.wristAlgaeL2)
        );
    }
}
