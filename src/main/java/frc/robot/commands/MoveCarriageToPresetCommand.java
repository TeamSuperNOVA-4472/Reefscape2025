package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.objectmodels.CarriagePreset;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorCarriageSubsystem;

/**
 * Moves the carriage to a specified preset. The elevator is unmoved.
 * If you wish to move the elevator, invoke the `moveToLevelSafe` command before this one.
 */
public class MoveCarriageToPresetCommand extends Command
{
    public static final double kArmTolerence = 5.0;
    public static final double kWristTolerance = 5.0;

    private final ElevatorCarriageSubsystem mElevatorCarriageSubsystem;

    private final Supplier<CarriagePreset> presetSupplier;

    public MoveCarriageToPresetCommand(CarriagePreset preset)
    {
        this(() -> preset);
    }

    public MoveCarriageToPresetCommand(Supplier<CarriagePreset> preset)
    {
        presetSupplier = preset;
        mElevatorCarriageSubsystem = ElevatorCarriageSubsystem.kInstance;
        addRequirements(mElevatorCarriageSubsystem);
    }

    @Override
    public void initialize() 
    {
        // Why do we need this as a supplier?
        mElevatorCarriageSubsystem.setPreset(presetSupplier.get());
    }

    @Override
    public boolean isFinished() 
    {
        double armDist = mElevatorCarriageSubsystem.getArmAngle() - mElevatorCarriageSubsystem.getArmSetpoint();
        double wristDist = mElevatorCarriageSubsystem.getWristAngle() - mElevatorCarriageSubsystem.getWristSetpoint();

        return Math.abs(armDist) < kArmTolerence && Math.abs(wristDist) < kWristTolerance;
    }
}
