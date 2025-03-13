package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeTeleop extends Command
{
    private final IntakeSubsystem mIntakeSubsystem;
    Supplier<Boolean> mIntakeButton;
    Supplier<Boolean> mOuttakeButton;
    public IntakeTeleop(Supplier<Boolean> pIntakeButton, Supplier<Boolean> pOuttakeButton)
    {
        mIntakeButton = pIntakeButton;
        mOuttakeButton = pOuttakeButton;
        mIntakeSubsystem = IntakeSubsystem.kInstance;
        addRequirements(mIntakeSubsystem);
    }

    @Override
    public void execute()
    {
        // if(mIntakeButton.get())
        // {
        //     mIntakeSubsystem.inTake();
        // }
        // else if(mOuttakeButton.get())
        // {
        //     mIntakeSubsystem.outTake();
        // }
    }
}
