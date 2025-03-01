package frc.robot.commands.Presets;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveCarriageToPresetCommand;
import frc.robot.commands.MoveToLevelCommand;
import frc.robot.commands.moveToLevelSafe;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AlgaeGround extends SequentialCommandGroup
{
    public AlgaeGround(CarriageSubsystem pCarriage, ElevatorSubsystem pElevator, IntakeSubsystem pIntake)
    {
        addCommands(
            new moveToLevelSafe(pCarriage, pElevator, pIntake, ElevatorSubsystem.kPresetGround),
            new MoveCarriageToPresetCommand(pCarriage, CarriageSubsystem.armAlgaeGround, CarriageSubsystem.wristAlgaeGround)
        );
    }    
}

