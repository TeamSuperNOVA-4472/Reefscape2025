package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Presets.CoralL2Preset;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ScoreLevel2 extends SequentialCommandGroup
{
    public ScoreLevel2(ElevatorSubsystem pElevatorSubsystem, CarriageSubsystem pCarriageSubsystem, IntakeSubsystem pIntakeSubsystem)
    {

        addCommands(new InstantCommand(() -> System.out.print("Hello")),new CoralL2Preset(pElevatorSubsystem, pCarriageSubsystem, pIntakeSubsystem),
        new InstantCommand(() -> pIntakeSubsystem.outtakeCoral()));
    }
}
