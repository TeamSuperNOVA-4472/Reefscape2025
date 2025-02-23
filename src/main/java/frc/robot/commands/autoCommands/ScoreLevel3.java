package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Presets.CoralL3Preset;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ScoreLevel3 extends SequentialCommandGroup
{
    public ScoreLevel3(ElevatorSubsystem pElevatorSubsystem, CarriageSubsystem pCarriageSubsystem, IntakeSubsystem pIntakeSubsystem)
    {

        addCommands(new InstantCommand(() -> System.out.print("Hello")),new CoralL3Preset(pElevatorSubsystem, pCarriageSubsystem),
        new InstantCommand(() -> pIntakeSubsystem.outtakeCoral()));
    }
}
