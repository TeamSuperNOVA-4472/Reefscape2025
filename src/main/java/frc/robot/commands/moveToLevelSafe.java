package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class moveToLevelSafe extends SequentialCommandGroup
{
    public moveToLevelSafe(CarriageSubsystem pCarriageSubsystem, ElevatorSubsystem pElevatorSubsystem, IntakeSubsystem pIntakeSubsystem, double pNewLevel) 
    {
        addRequirements(pCarriageSubsystem, pElevatorSubsystem);
        addCommands(new moveOutOfDangerZone(pCarriageSubsystem, pIntakeSubsystem), new MoveToLevelCommand(pElevatorSubsystem, pNewLevel));
    }
}
