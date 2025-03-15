package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.objectmodels.CarriagePreset;
import frc.robot.subsystems.ElevatorCarriageSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * This command moves the elevator and carriage to a new preset. Handles
 * safely moving the carriage automatically, no need to worry about that.
 */
public class SwitchPresetCommand extends SequentialCommandGroup
{
    public static SwitchPresetCommand stow(boolean keepAlive)
    {
        return new SwitchPresetCommand(true, keepAlive);
    }

    private final ElevatorCarriageSubsystem mElevatorCarriage;
    private final IntakeSubsystem mIntake;

    private final Supplier<CarriagePreset> mPresetSupplier;

    private CarriagePreset safePosition, elevatorPosition, wristPosition, armPosition;

    public SwitchPresetCommand(CarriagePreset newPreset, boolean keepAlive)
    {
        // Same thing as the other constructor but with a simple constant supplier.
        this(() -> newPreset, keepAlive);
    }
    public SwitchPresetCommand(Supplier<CarriagePreset> newPreset, boolean keepAlive)
    {
        mElevatorCarriage = ElevatorCarriageSubsystem.kInstance;
        mIntake = IntakeSubsystem.kInstance;
        mPresetSupplier = newPreset;

        addCommands(
            new InstantCommand(this::init),                // Run initial setup.
            new MovePresetCommand(() -> safePosition),     // Move to the safe position.
            new MovePresetCommand(() -> elevatorPosition), // Move the elevator.
            new MovePresetCommand(() -> wristPosition),    // Move the wrist. Might be worth combining.
            new MovePresetCommand(() -> armPosition)       // Move the arm.
        );
        if (keepAlive) addCommands(new ForeverCommand());
        addRequirements(mElevatorCarriage); // Not actively modifying intake.
    }
    private SwitchPresetCommand(boolean justStow, boolean keepAlive) // Just stow. Nothing else.
    {
        mElevatorCarriage = ElevatorCarriageSubsystem.kInstance;
        mIntake = IntakeSubsystem.kInstance;
        mPresetSupplier = null; // Unused in this context. Careful!

        addCommands(
            new InstantCommand(this::initStowOnly),       // Run initial setup.
            new MovePresetCommand(() -> safePosition),    // Move to the safe position.
            new MovePresetCommand(() -> elevatorPosition) // Move the elevator.
        );
        if (keepAlive) addCommands(new ForeverCommand());
        addRequirements(mElevatorCarriage); // Not actively modifying intake.
    }

    private void init()
    {
        CarriagePreset newPreset = mPresetSupplier.get();

        if (mIntake.hasAlgae()) safePosition = CarriagePreset.kStowAlgae;
        else safePosition = CarriagePreset.kStowCoral;

        safePosition = safePosition.withElevatorPreset(mElevatorCarriage.getElevatorHeight());
        elevatorPosition = safePosition.withElevatorPreset(newPreset);
        wristPosition = elevatorPosition.withWristPreset(newPreset);
        armPosition = wristPosition.withArmPreset(newPreset);

        mElevatorCarriage.setDesiredPreset(newPreset);
    }
    private void initStowOnly()
    {
        CarriagePreset stowPosition;
        if (mIntake.hasAlgae()) stowPosition = CarriagePreset.kStowAlgae;
        else stowPosition = CarriagePreset.kStowCoral;

        safePosition = stowPosition.withElevatorPreset(mElevatorCarriage.getElevatorHeight());
        elevatorPosition = stowPosition;

        // Redundant
        wristPosition = stowPosition;
        armPosition = stowPosition;
        
        mElevatorCarriage.setDesiredPreset(stowPosition);
    }

    @Override
    public void cancel()
    {
        mElevatorCarriage.emptyDesiredPreset();
        super.cancel();
    }

    // Unsafe preset command. Just runs `setPreset()` and waits for completion.
    private static class MovePresetCommand extends Command
    {
        private final ElevatorCarriageSubsystem mElevatorCarriage;
        private final Supplier<CarriagePreset> mPresetSupplier;

        private CarriagePreset activePreset;

        public MovePresetCommand(Supplier<CarriagePreset> newPreset)
        {
            mPresetSupplier = newPreset;
            mElevatorCarriage = ElevatorCarriageSubsystem.kInstance;
        }

        @Override
        public void initialize()
        {
            activePreset = mPresetSupplier.get();
            mElevatorCarriage.setPreset(activePreset);
        }

        @Override
        public void execute()
        {            
            //mElevatorCarriage.setPreset(activePreset);
        }   

        @Override
        public boolean isFinished()
        {
            final double armDelta = 10, // 1 or 2
                         wristDelta = 10,
                         elevatorDelta = 1;
            
            return Math.abs(mElevatorCarriage.getArmAngle() - activePreset.kArmPreset) <= armDelta &&
                   Math.abs(mElevatorCarriage.getWristAngle() - activePreset.kWristPreset) <= wristDelta &&
                   Math.abs(mElevatorCarriage.getElevatorHeight() - activePreset.kElevatorPreset) <= elevatorDelta;
        }
    }
}
