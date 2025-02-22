package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.objectmodels.IntakePreset;
import frc.robot.subsystems.CarriageSubsystem;

public class AlgaeIntakePreset extends SequentialCommandGroup
{
    public AlgaeIntakePreset(CarriageSubsystem pCarriage)
    {
        addCommands(
            new MoveCarriageToPresetCommand(pCarriage, IntakePreset.kLoadAlgae)
        );
    }    
}
