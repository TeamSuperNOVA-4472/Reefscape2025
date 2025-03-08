package frc.robot.commands.Presets;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Moves the robot to The L2 Algae position and begin intaking.
 */
public class AlgaeL2IntakePreset extends SequentialCommandGroup
{
    public AlgaeL2IntakePreset()
    {
        addCommands(
            new AlgaeL2(),
            new InstantCommand(
                () -> CarriageSubsystem.instance().setAlgaeMode(true)
            )
        );
    }    
}
