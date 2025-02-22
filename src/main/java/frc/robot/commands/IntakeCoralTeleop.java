package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import javax.print.StreamPrintService;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

    private Supplier<Boolean> mLoadButton;
    private Supplier<Boolean> mL1Button;
    private Supplier<Boolean> mL2Button;
    private Supplier<Boolean> mL3Button;
    private Supplier<Boolean> mL4Button;

    public IntakeCoralTeleop(
        IntakeSubsystem pIntakeSubsystem,
        CarriageSubsystem pCarriageSubsystem,
        ElevatorSubsystem pElevatorSubsytem,
        Supplier<Boolean> pLoadButton, 
        Supplier<Boolean> pL1Button,
        Supplier<Boolean> pL2Button,
        Supplier<Boolean> pL3Button,
        Supplier<Boolean> pL4Button)
    {
        mLoadButton = pLoadButton;
        mL1Button = pL1Button;
        mL2Button = pL2Button;
        mL3Button = pL3Button;
        mL4Button = pL4Button;

        //TODO: define default arm preset
        //TODO: define default wrist preset

        mIntakeSubsystem = pIntakeSubsystem;
        mCarriageSubsystem = pCarriageSubsystem;
        mElevatorSubsystem = pElevatorSubsytem;

        addRequirements(mIntakeSubsystem, mCarriageSubsystem, mElevatorSubsystem);
    }

    private Optional<IntakePreset> getPreset()
    {
        if (mLoadButton.get())
        {
            //TODO: RETURN MATCH LOAD
            return Optional.of(IntakePreset.kScoreL1);
        }
        else if (mL1Button.get())
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
            //TODO: RETURN L4 PRESET
            return Optional.of(IntakePreset.kScoreL3);
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
