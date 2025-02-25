package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.objectmodels.IntakePreset;
import frc.robot.subsystems.CarriageSubsystem;

public class AlgaeIntakeTeleop extends SequentialCommandGroup
{
    public AlgaeIntakeTeleop(CarriageSubsystem pCarriage)
    {
        addCommands(
            new MoveCarriageToPresetCommand(pCarriage, IntakePreset.kLoadAlgae)
        );
    }    
}
