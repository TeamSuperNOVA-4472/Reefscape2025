package frc.robot.commands.tester;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.objectmodels.IntakePresets;
import frc.robot.subsystems.CarriageSubsystem;

public class CarriageTester extends Command 
{
    private final Supplier<Double> mWristVoltage;
    private final Supplier<Double> mElbowVoltage;
    private final CarriageSubsystem mCarriageSubsystem;

    public CarriageTester(Supplier<Double> pWristVoltage, Supplier<Double> pElbowVoltage, CarriageSubsystem pCarriageSubsystem)
    {
        mElbowVoltage = pElbowVoltage;
        mWristVoltage = pWristVoltage;
        mCarriageSubsystem = pCarriageSubsystem;
        addRequirements(mCarriageSubsystem);
    }
    
    @Override
    public void initialize() {
        //mCarriageSubsystem.stop();
        mCarriageSubsystem.setActivePreset(IntakePresets.kAway);
        mCarriageSubsystem.resetWristPID();
    }

    @Override
    public void execute()
    {
        //mCarriageSubsystem.setArmVoltage(mElbowVoltage.get()*6);
        //mCarriageSubsystem.setWristVoltage(mWristVoltage.get()*6);

    }
}
