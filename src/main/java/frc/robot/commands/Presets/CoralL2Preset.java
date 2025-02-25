package frc.robot.commands.Presets;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveCarriageToPresetCommand;
import frc.robot.commands.MoveToLevelCommand;
import frc.robot.commands.moveToLevelSafe;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class CoralL2Preset extends SequentialCommandGroup
{
    public CoralL2Preset(ElevatorSubsystem pElevator, CarriageSubsystem pCarriage, IntakeSubsystem pIntakeSubsystem)
    {
        addCommands(
            new moveToLevelSafe(pCarriage, pElevator, pIntakeSubsystem, ElevatorSubsystem.kPresetL2),
            new MoveCarriageToPresetCommand(pCarriage, CarriageSubsystem.armPresetL2, CarriageSubsystem.wristPresetL2)
        );
    }    
}
