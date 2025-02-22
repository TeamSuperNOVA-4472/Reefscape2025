package frc.robot.commands.Presets;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveCarriageToPresetCommand;
import frc.robot.commands.MoveToLevelCommand;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class CoralL1Preset extends SequentialCommandGroup
{
    public CoralL1Preset(ElevatorSubsystem pElevator, CarriageSubsystem pCarriage)
    {
        addCommands(
            new StowCarriagePosition(pCarriage, pElevator),
            new MoveToLevelCommand(pElevator, ElevatorSubsystem.kPresetL1),
            new MoveCarriageToPresetCommand(pCarriage, CarriageSubsystem.armPresetL1, CarriageSubsystem.wristPresetL1)
        );
    }    
}
