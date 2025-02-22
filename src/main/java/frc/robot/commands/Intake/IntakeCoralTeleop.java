package frc.robot.commands.Intake;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import javax.print.StreamPrintService;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.CarriageAndElevatorCommand;
import frc.robot.objectmodels.IntakePreset;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorCarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCoralTeleop extends Command
{
    private final IntakeSubsystem mIntakeSubsystem;
    private final CarriageSubsystem mCarriageSubsystem;
    private final ElevatorSubsystem mElevatorSubsystem;

    private CarriageAndElevatorCommand mCommand;

    private Supplier<Boolean> mL1Button;
    private Supplier<Boolean> mL2Button;
    private Supplier<Boolean> mL3Button;
    private Supplier<Boolean> mL4Button;

    public IntakeCoralTeleop(
        IntakeSubsystem pIntakeSubsystem,
        CarriageSubsystem pCarriageSubsystem,
        ElevatorSubsystem pElevatorSubsytem,
        Supplier<Boolean> pL1Button,
        Supplier<Boolean> pL2Button,
        Supplier<Boolean> pL3Button,
        Supplier<Boolean> pL4Button)
    {
        mL1Button = pL1Button;
        mL2Button = pL2Button;
        mL3Button = pL3Button;
        mL4Button = pL4Button;

        mIntakeSubsystem = pIntakeSubsystem;
        mCarriageSubsystem = pCarriageSubsystem;
        mElevatorSubsystem = pElevatorSubsytem;

        addRequirements(mIntakeSubsystem, mCarriageSubsystem, mElevatorSubsystem);
    }

    private Optional<IntakePreset> getPreset()
    {
        if (mL1Button.get())
        {
            return Optional.of(IntakePreset.kScoreL1);
        }
        else if (mL2Button.get())
        {
            return Optional.of(IntakePreset.kScoreL2);
        }
        else if (mL3Button.get())
        {
            return Optional.of(IntakePreset.kScoreL3);
        }
        else if (mL4Button.get())
        {
            return Optional.of(IntakePreset.kScoreL4);
        }
        else
        {
            return Optional.empty();
        }
    }

    @Override
    public void execute()
    {
        Optional<IntakePreset> preset = getPreset();
        if (preset.isPresent())
        {
            mCommand = new CarriageAndElevatorCommand(mCarriageSubsystem, mElevatorSubsystem, preset.get());
            mCommand.schedule();
        }
    }
}
