package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.MoveCarriageToPresetCommand;
import frc.robot.commands.Intake.MoveToLevelCommand;
import frc.robot.objectmodels.IntakePreset;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class StowCarriagePositionAlgae extends SequentialCommandGroup 
{
    public StowCarriagePositionAlgae(CarriageSubsystem pCarriage, ElevatorSubsystem pElevator)
    {
        addCommands(
            new MoveToLevelCommand(pElevator, IntakePreset.kMovingAlgae),
            new MoveCarriageToPresetCommand(pCarriage, IntakePreset.kMovingAlgae)
        );
    }    
}