package frc.robot.commands.tester;


import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeTester extends Command {
    private final Supplier<Boolean> mCoralIntake;
    private final Supplier<Boolean> mCoralOuttake;
    private final Supplier<Boolean> mAlgaeIntake;
    private final Supplier<Boolean> mAlgaeOuttake;
    private final IntakeSubsystem mIntakeSubsystem;
    
    public IntakeTester(
        Supplier<Boolean> pCoralIntake,
        Supplier<Boolean> pCoralOuttake,
        Supplier<Boolean> pAlgaeIntake,
        Supplier<Boolean> pAlgaeOuttake,
        IntakeSubsystem pIntakeSubsystem)
    {
        mCoralIntake = pCoralIntake;
        mCoralOuttake = pCoralOuttake;
        mAlgaeIntake = pAlgaeIntake;
        mAlgaeOuttake = pAlgaeOuttake;
        mIntakeSubsystem = pIntakeSubsystem;
        addRequirements(mIntakeSubsystem);
    }

    @Override
    public void initialize() {
        mIntakeSubsystem.stop();
    }

    @Override
    public void execute() {
        if(mAlgaeIntake.get()) {
            mIntakeSubsystem.intakeAlgae();
        }
        else if(mAlgaeOuttake.get()) {
            mIntakeSubsystem.outtakeAlgae();
        }
        else if(mCoralIntake.get()) {
            mIntakeSubsystem.intakeCoral();
        }
        else if(mCoralOuttake.get()) {
            mIntakeSubsystem.outtakeCoral();
        } else {
            mIntakeSubsystem.stop();
        }
    }
}
