package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.objectmodels.CarriagePreset;
import frc.robot.subsystems.CarriageSubsystem;

/**
 * Moves the carriage to a specified preset. The elevator is unmoved.
 * If you wish to move the elevator, invoke the `moveToLevelSafe` command before this one.
 */
public class MoveCarriageToPresetCommand extends Command
{
    public static final double kArmTolerence = 5.0;
    public static final double kWristTolerance = 5.0;

    private final CarriageSubsystem mCarriageSubsystem;

    private final Supplier<CarriagePreset> presetSupplier;

    public MoveCarriageToPresetCommand(CarriagePreset preset)
    {
        this(() -> preset);
    }

    public MoveCarriageToPresetCommand(Supplier<CarriagePreset> preset)
    {
        presetSupplier = preset;
        mCarriageSubsystem = CarriageSubsystem.kInstance;
        addRequirements(mCarriageSubsystem);
    }

    @Override
    public void initialize() 
    {
        // Why do we need this as a supplier?
        mCarriageSubsystem.setPreset(presetSupplier.get());
    }

    @Override
    public boolean isFinished() 
    {
        double armDist = mCarriageSubsystem.getArmAngle() - mCarriageSubsystem.getArmSetpoint();
        double wristDist = mCarriageSubsystem.getWristAngle() - mCarriageSubsystem.getWristSetpoint();

        return Math.abs(armDist) < kArmTolerence && Math.abs(wristDist) < kWristTolerance;
    }
}
