package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class moveToLevelSafe extends SequentialCommandGroup
{
    double armPreset;
    double wristPreset;

    public moveToLevelSafe(CarriageSubsystem pCarriageSubsystem, ElevatorSubsystem pElevatorSubsystem, double pNewLevel) 
    {
        addRequirements(pCarriageSubsystem, pElevatorSubsystem);

        armPreset = pCarriageSubsystem.getArmSetpoint();

        wristPreset = pCarriageSubsystem.getWristSetpoint();

        addCommands(new moveOutOfDangerZone(pCarriageSubsystem), new MoveToLevelCommand(pElevatorSubsystem, pNewLevel), new MoveCarriageToPresetCommand(pCarriageSubsystem, armPreset, wristPreset));
    }
}
