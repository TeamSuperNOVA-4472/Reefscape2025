package frc.robot.commands.Presets;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveCarriageToPresetCommand;
import frc.robot.commands.MoveToLevelCommand;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class CoralL4Preset extends SequentialCommandGroup
{
    public CoralL4Preset(ElevatorSubsystem pElevator, CarriageSubsystem pCarriage)
    {
        addCommands(
            new StowCarriagePosition(pCarriage, pElevator),
            new MoveToLevelCommand(pElevator, ElevatorSubsystem.kPresetL4),
            new MoveCarriageToPresetCommand(pCarriage, CarriageSubsystem.armPresetL4, CarriageSubsystem.wristPresetL4)
        );
    }      
}
