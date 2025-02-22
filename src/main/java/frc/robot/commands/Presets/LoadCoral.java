package frc.robot.commands.Presets;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveCarriageToPresetCommand;
import frc.robot.commands.MoveToLevelCommand;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class LoadCoral extends SequentialCommandGroup
{
    public LoadCoral(ElevatorSubsystem pElevator, CarriageSubsystem pCarriage)
    {
        addCommands(
            new MoveCarriageToPresetCommand(pCarriage, CarriageSubsystem.armMovingAngle, CarriageSubsystem.wristMovingAngle),
            new MoveToLevelCommand(pElevator, ElevatorSubsystem.initialHeight),
            new MoveCarriageToPresetCommand(pCarriage, CarriageSubsystem.armMovingAngle, CarriageSubsystem.wristCoralLoad),
            new MoveCarriageToPresetCommand(pCarriage, CarriageSubsystem.armCoralLoad, CarriageSubsystem.wristCoralLoad),
            new MoveToLevelCommand(pElevator, ElevatorSubsystem.kPresetCoralPickup)
        );
    }
}