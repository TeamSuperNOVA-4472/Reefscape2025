package frc.robot.commands.tester;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorTester extends Command {
    private final ElevatorSubsystem mElevator;
    private final Supplier<Double> mElevatorSpeed;
    public ElevatorTester(ElevatorSubsystem pElevator, Supplier<Double> pElevatorSpeed) {
        mElevator = pElevator;
        mElevatorSpeed = pElevatorSpeed;
        addRequirements(mElevator);
        mElevator.resetEncoder();
    }

    @Override
    public void initialize() {
        mElevator.stop();
        mElevator.resetElevatorPID();
        //mElevator.setPreset(ElevatorSubsystem.initialHeight);
    }

    @Override
    public void execute() {
        mElevator.setManualVoltage(mElevatorSpeed.get()*6);
    }
}
