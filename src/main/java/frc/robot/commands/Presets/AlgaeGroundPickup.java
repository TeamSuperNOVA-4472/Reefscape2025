package frc.robot.commands.Presets;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveCarriageToPresetCommand;
import frc.robot.commands.MoveToLevelCommand;
import frc.robot.commands.moveToLevelSafe;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// Why was this not already here? The presets literally were already defined.
public class AlgaeGroundPickup extends SequentialCommandGroup
{
    public AlgaeGroundPickup(ElevatorSubsystem pElevator, CarriageSubsystem pCarriage, IntakeSubsystem pIntakeSubsystem)
    {
        addCommands(
            new moveToLevelSafe(pCarriage, pElevator, pIntakeSubsystem, ElevatorSubsystem.kPresetGroundPickup),
            new MoveCarriageToPresetCommand(pCarriage, CarriageSubsystem.armPresetGroundPickup, CarriageSubsystem.wristPresetGroundPickup)
        );
    }    
}
