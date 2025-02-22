package frc.robot.commands.Presets;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveCarriageToPresetCommand;
import frc.robot.subsystems.CarriageSubsystem;

public class AlgaeIntakePreset extends SequentialCommandGroup
{
    public AlgaeIntakePreset(CarriageSubsystem pCarriage)
    {
        addCommands(
            new MoveCarriageToPresetCommand(pCarriage, CarriageSubsystem.armAlgaeLoad, CarriageSubsystem.wristAlgaeLoad)
        );
    }    
}
