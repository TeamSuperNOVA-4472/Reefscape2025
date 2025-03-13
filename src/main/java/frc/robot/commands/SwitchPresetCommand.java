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
    public static SwitchPresetCommand stow()
    {
        // Not sure if I want to make it a final field here, as
        // passing the same reference might cause issues. This
        // always makes a new reference.
        return new SwitchPresetCommand(true);
    }

    private final ElevatorCarriageSubsystem mElevatorCarriage;
    private final IntakeSubsystem mIntake;

    private final Supplier<CarriagePreset> mPresetSupplier;

    private CarriagePreset safePosition, elevatorPosition, wristPosition, armPosition;

    public SwitchPresetCommand(CarriagePreset newPreset)
    {
        // Same thing as the other constructor but with a simple constant supplier.
        this(() -> newPreset);
    }
    public SwitchPresetCommand(Supplier<CarriagePreset> newPreset)
    {
        mElevatorCarriage = ElevatorCarriageSubsystem.kInstance;
        mIntake = IntakeSubsystem.kInstance;
        mPresetSupplier = newPreset;

        addCommands(
            new InstantCommand(this::init),                // Run initial setup.
            new MovePresetCommand(() -> safePosition),     // Move to the safe position.
            new MovePresetCommand(() -> elevatorPosition), // Move the elevator.
            new MovePresetCommand(() -> wristPosition),    // Move the wrist.
            new MovePresetCommand(() -> armPosition),      // Move the arm.
            new InstantCommand(() -> System.out.println("[SWITCH] Done"))
        );
        addRequirements(mElevatorCarriage); // Not actively modifying intake.

        System.out.println("YEAH " + mElevatorCarriage.getElevatorHeight());
    }
    private SwitchPresetCommand(boolean justStow) // Just stow. Nothing else.
    {
        mElevatorCarriage = ElevatorCarriageSubsystem.kInstance;
        mIntake = IntakeSubsystem.kInstance;
        mPresetSupplier = null; // Unused in this context. Careful!

        addCommands(
            new InstantCommand(this::initStowOnly),       // Run initial setup.
            new MovePresetCommand(() -> safePosition),    // Move to the safe position.
            new MovePresetCommand(() -> elevatorPosition), // Move the elevator.
            new InstantCommand(() -> System.out.println("[SWITCH] Done"))
        );
        addRequirements(mElevatorCarriage); // Not actively modifying intake.
    }

    private void init()
    {
        System.out.println("[SWITCH] Initializing.");
        CarriagePreset newPreset = mPresetSupplier.get();

        if (mIntake.hasAlgae()) safePosition = CarriagePreset.kStowAlgae;
        else safePosition = CarriagePreset.kStowCoral;

        safePosition = safePosition.withElevatorPreset(mElevatorCarriage.getElevatorHeight());
        elevatorPosition = safePosition.withElevatorPreset(newPreset);
        wristPosition = elevatorPosition.withWristPreset(newPreset);
        armPosition = wristPosition.withArmPreset(newPreset);
    }
    private void initStowOnly()
    {
        System.out.println("[SWITCH] Initializing (stow).");
        CarriagePreset stowPosition;
        if (mIntake.hasAlgae()) stowPosition = CarriagePreset.kStowAlgae;
        else stowPosition = CarriagePreset.kStowCoral;

        safePosition = stowPosition.withElevatorPreset(mElevatorCarriage.getElevatorHeight());
        elevatorPosition = stowPosition;

        // Redundant
        wristPosition = stowPosition;
        armPosition = stowPosition;
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

            double deltaArm = mElevatorCarriage.getArmAngle() - activePreset.kArmPreset;
            double deltaWrist = mElevatorCarriage.getWristAngle() - activePreset.kWristPreset;
            double deltaElev = mElevatorCarriage.getElevatorHeight() - activePreset.kElevatorPreset;
            System.out.println("[SWITCH] Beginning move sequence.");
            System.out.println("[SWITCH] Delta Arm: " + deltaArm + ", Delta Wrist: " + deltaWrist + ", Delta Height" + deltaElev);
        }

        @Override
        public void execute()
        {            
            mElevatorCarriage.setPreset(activePreset);
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
