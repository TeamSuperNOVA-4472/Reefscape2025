package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class moveToLevelSafe extends SequentialCommandGroup
{
    public moveToLevelSafe(CarriageSubsystem pCarriageSubsystem, ElevatorSubsystem pElevatorSubsystem, double pNewLevel) 
    {
        addRequirements(pCarriageSubsystem, pElevatorSubsystem);

        addCommands(new moveOutOfDangerZone(pCarriageSubsystem), new MoveToLevelCommand(pElevatorSubsystem, pNewLevel));
    }
}
