package frc.robot.commands.Presets;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Moves the robot to The L3 Algae position and begin intaking.
 */
public class AlgaeL3IntakePreset extends SequentialCommandGroup
{
    public AlgaeL3IntakePreset()
    {
        addCommands(
            new AlgaeL3(),
            new InstantCommand(
                () -> CarriageSubsystem.kInstance.setAlgaeMode(true)
            )
        );
    }    
}
