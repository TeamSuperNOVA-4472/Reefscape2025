package frc.robot.commands.Presets;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveCarriageToPresetCommand;
import frc.robot.commands.MoveToLevelCommand;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class StowCarriagePosition extends SequentialCommandGroup 
{
    public StowCarriagePosition(CarriageSubsystem pCarriage, ElevatorSubsystem pElevator)
    {
        addCommands(
            new MoveToLevelCommand(pElevator, ElevatorSubsystem.initialHeight),
            new MoveCarriageToPresetCommand(pCarriage, CarriageSubsystem.armMovingAngle, CarriageSubsystem.wristMovingAngle),
            new InstantCommand(
                () -> pCarriage.setAlgaeMode(true)
            )
        );
    }    
}
