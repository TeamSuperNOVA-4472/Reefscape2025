package frc.robot.commands.Presets;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveCarriageToPresetCommand;
import frc.robot.commands.MoveToLevelCommand;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class StowCarriagePositionAlgae extends SequentialCommandGroup 
{
    public StowCarriagePositionAlgae(CarriageSubsystem pCarriage, ElevatorSubsystem pElevator)
    {
        addCommands(
            new MoveCarriageToPresetCommand(pCarriage, CarriageSubsystem.armAlgaeStow, CarriageSubsystem.wristAlgaeStow),
            new MoveToLevelCommand(pElevator, ElevatorSubsystem.initialHeight)
        );
    }    
}