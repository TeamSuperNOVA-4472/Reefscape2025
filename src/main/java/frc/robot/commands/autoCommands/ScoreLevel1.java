package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Presets.CoralL1Preset;
import frc.robot.commands.Presets.CoralL3Preset;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ScoreLevel1 extends SequentialCommandGroup
{
    public ScoreLevel1(ElevatorSubsystem pElevatorSubsystem, CarriageSubsystem pCarriageSubsystem, IntakeSubsystem pIntakeSubsystem)
    {

        addCommands(
            //new CoralL1Preset(pElevatorSubsystem, pCarriageSubsystem, pIntakeSubsystem),
        new InstantCommand(() -> pIntakeSubsystem.outtakeCoral()));
    }
}
