package frc.robot.commands.Presets;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveCarriageToPresetCommand;
import frc.robot.commands.MoveToLevelCommand;
import frc.robot.commands.moveToLevelSafe;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ArmBackPreset extends SequentialCommandGroup
{
    public ArmBackPreset(ElevatorSubsystem pElevator, CarriageSubsystem pCarriage, IntakeSubsystem pIntakeSubsystem)
    {
        addCommands(
            new moveToLevelSafe(pCarriage, pElevator, pIntakeSubsystem, ElevatorSubsystem.initialHeight),
            new MoveCarriageToPresetCommand(pCarriage, CarriageSubsystem.armAlgaeStow, 10.0),
            new MoveCarriageToPresetCommand(pCarriage, 95.0, 10.0)
        );
    }    
}
