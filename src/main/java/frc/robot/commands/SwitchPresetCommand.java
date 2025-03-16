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
        SwitchPresetCommand preset = new SwitchPresetCommand(true);
        preset.addCommands(
            new InstantCommand(preset::initStowOnly),         // Run initial setup.
            new MovePresetCommand(() -> preset.posPoints[0]), // Move to the safe position.
            new MovePresetCommand(() -> preset.posPoints[1])  // Move the elevator.
        );
        if (keepAlive) preset.addCommands(new ForeverCommand());
        return preset;
    }
    public static SwitchPresetCommand load(boolean keepAlive)
    {
        SwitchPresetCommand preset = new SwitchPresetCommand(true);
        preset.addCommands(
            new InstantCommand(preset::initCoralLoad),        // Run initial setup
            new MovePresetCommand(() -> preset.posPoints[0]), // Move to the safe position.
            new MovePresetCommand(() -> preset.posPoints[1]), // Move the elevator to the bottom.
            new MovePresetCommand(() -> preset.posPoints[2]), // Move the wrist to load position.
            new MovePresetCommand(() -> preset.posPoints[3]), // Move the arm to load position.
            new MovePresetCommand(() -> preset.posPoints[4])  // Move the elevator to load position.
        );
        if (keepAlive) preset.addCommands(new ForeverCommand());
        return preset;
    }

    private final ElevatorCarriageSubsystem mElevatorCarriage;
    private final IntakeSubsystem mIntake;

    private final Supplier<CarriagePreset> mPresetSupplier;

    private CarriagePreset[] posPoints;

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
            new InstantCommand(this::init),            // Run initial setup.
            new MovePresetCommand(() -> posPoints[0]), // Move to the safe position.
            new MovePresetCommand(() -> posPoints[1]), // Move the elevator.
            new MovePresetCommand(() -> posPoints[2]), // Move the wrist. Might be worth combining.
            new MovePresetCommand(() -> posPoints[3])  // Move the arm.
        );
        if (keepAlive) addCommands(new ForeverCommand());
        addRequirements(mElevatorCarriage); // Not actively modifying intake.
    }
    private SwitchPresetCommand(boolean special) // Do custom constructor things. Nothing else.
    {
        // Anything using this constructor should do some defining of its own.
        // Used in the static methods above.
        mElevatorCarriage = ElevatorCarriageSubsystem.kInstance;
        mIntake = IntakeSubsystem.kInstance;
        mPresetSupplier = null; // Unused in this context. Careful!
        addRequirements(mElevatorCarriage); // Not actively modifying intake.
    }

    private void init()
    {
        CarriagePreset newPreset = mPresetSupplier.get();
        if (newPreset == CarriagePreset.kStowAlgae ||
            newPreset == CarriagePreset.kStowCoral ||
            newPreset == CarriagePreset.kCoralLoad)
        {
            System.err.println("[SWITCH] PLEASE DO NOT PASS A STOW OR A LOADING PRESET INTO THE SWITCHPRESETCOMMAND CONSTRUCTOR!!! If you are stowing, call .stow(). If you are loading, call load().");
            cancel();
        }

        CarriagePreset safePosition;
        if (mIntake.hasAlgae()) safePosition = CarriagePreset.kStowAlgae;
        else safePosition = CarriagePreset.kStowCoral;

        safePosition = safePosition.withElevatorPreset(mElevatorCarriage.getElevatorHeight());
        CarriagePreset elevatorPosition = safePosition.withElevatorPreset(newPreset);
        CarriagePreset wristPosition = elevatorPosition.withWristPreset(newPreset);
        CarriagePreset armPosition = wristPosition.withArmPreset(newPreset);

        posPoints = new CarriagePreset[]
        {
            safePosition,     // Arm & Wrist to stow angle.
            elevatorPosition, // Elevator to final position
            wristPosition,    // Wrist to final position
            armPosition       // Arm to final position
        };

        mElevatorCarriage.setDesiredPreset(newPreset);
    }
    private void initStowOnly()
    {
        CarriagePreset stowPosition;
        if (mIntake.hasAlgae()) stowPosition = CarriagePreset.kStowAlgae;
        else stowPosition = CarriagePreset.kStowCoral;

        CarriagePreset safePosition = stowPosition.withElevatorPreset(mElevatorCarriage.getElevatorHeight());
        posPoints = new CarriagePreset[]
        {
            safePosition, // Arm & Wrist to stow angle.
            stowPosition  // Elevator to stow height.
        };
        
        mElevatorCarriage.setDesiredPreset(stowPosition);
    }
    private void initCoralLoad()
    {
        CarriagePreset loadPosition = CarriagePreset.kCoralLoad;

        // We need to go all the way down first before changing wrist stuff. Kind of a pain.
        CarriagePreset stowPosition;
        if (mIntake.hasAlgae()) stowPosition = CarriagePreset.kStowAlgae;
        else stowPosition = CarriagePreset.kStowCoral;

        CarriagePreset safePosition = stowPosition.withElevatorPreset(mElevatorCarriage.getElevatorHeight());
        CarriagePreset elevatorBottom = stowPosition;
        CarriagePreset wristPosition = stowPosition.withWristPreset(loadPosition);
        CarriagePreset armPosition = wristPosition.withArmPreset(loadPosition);
        posPoints = new CarriagePreset[]
        {
            safePosition,   // Move arm & wrist to the stow position (temporary).
            elevatorBottom, // Move elevator to bottom (temporary), so we have room to move wrist.
            wristPosition,  // Move wrist to load position.
            armPosition,    // Move arm to load position.
            loadPosition    // Finally, move elevator to load position.
        };
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
