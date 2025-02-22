package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.objectmodels.IntakePreset;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class StowCarriagePosition extends SequentialCommandGroup 
{
    public StowCarriagePosition(CarriageSubsystem pCarriage, ElevatorSubsystem pElevator)
    {
        addCommands(
            new MoveToLevelCommand(pElevator, IntakePreset.kMoving),
            new MoveCarriageToPresetCommand(pCarriage, IntakePreset.kMoving)
        );
    }    
}
