package frc.robot.commands.Presets;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveCarriageToPresetCommand;
import frc.robot.commands.MoveToLevelCommand;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class AlgaeProcessor extends SequentialCommandGroup {
    public AlgaeProcessor(ElevatorSubsystem pElevator, CarriageSubsystem pCarriage)
    {
        addCommands(
            new StowCarriagePositionAlgae(pCarriage, pElevator),
            new MoveToLevelCommand(pElevator, ElevatorSubsystem.kPresetProcessor),
            new MoveCarriageToPresetCommand(pCarriage, CarriageSubsystem.armAlgaeProcessor, CarriageSubsystem.wristAlgaeProcessor)
        );
    }    
}
