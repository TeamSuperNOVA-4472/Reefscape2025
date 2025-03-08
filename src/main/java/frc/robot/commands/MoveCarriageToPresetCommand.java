package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CarriageSubsystem;

public class MoveCarriageToPresetCommand extends Command
{
    public static final double kArmTolerence = 5.0;
    public static final double kWristTolerance = 5.0;

    private final CarriageSubsystem mCarriageSubsystem;

    private Double armPreset;
    private Double wristPreset;

    private final Supplier<Double> armSupply;
    private final Supplier<Double> wristSupply;

    public MoveCarriageToPresetCommand(CarriageSubsystem carriageSubsystem, Double armPreset, Double wristPreset)
    {
        this(carriageSubsystem, () -> armPreset, () -> wristPreset);
    }

    public MoveCarriageToPresetCommand(CarriageSubsystem carriageSubsystem, Supplier<Double> armPreset, Supplier<Double> wristPreset)
    {
        armSupply = armPreset;
        wristSupply = wristPreset;
        mCarriageSubsystem = carriageSubsystem;
        addRequirements(mCarriageSubsystem);
    }

    @Override
    public void initialize() 
    {
        armPreset = armSupply.get();
        wristPreset = wristSupply.get();
        mCarriageSubsystem.setArmPreset(armPreset);
        mCarriageSubsystem.setWristPreset(wristPreset);
    }

    @Override
    public boolean isFinished() 
    {
        double armDist = mCarriageSubsystem.getArmAngle() - mCarriageSubsystem.getArmSetpoint();
        double wristDist = mCarriageSubsystem.getWristAngle() - mCarriageSubsystem.getWristSetpoint();

        return Math.abs(armDist) < kArmTolerence && Math.abs(wristDist) < kWristTolerance;
    }
}
