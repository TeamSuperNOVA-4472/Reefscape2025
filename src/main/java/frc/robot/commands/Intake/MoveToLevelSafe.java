package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.objectmodels.IntakePreset;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class MoveToLevelSafe extends SequentialCommandGroup
{
    public MoveToLevelSafe(CarriageSubsystem pCarriageSubsystem, ElevatorSubsystem pElevatorSubsystem, IntakeSubsystem pIntakeSubsystem, IntakePreset pNewPreset) 
    {
        addRequirements(pCarriageSubsystem, pElevatorSubsystem);
        addCommands(new MoveOutOfDangerZone(pCarriageSubsystem, pIntakeSubsystem), new MoveToLevelCommand(pElevatorSubsystem, pNewPreset));
    }
}
