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
 * 
 * This command is slightly less safe than the previous iteration, but it
 * works faster and better in certain scenarios (namely L4 and the loading
 * preset). This command can NOT handle all possible scenarios!
 * 
 * REALLY BAD THINGS WILL HAPPEN IF THE CARRIAGE IS NOT EXACTLY WHERE IT
 * IS EXPECTED!!!
 */
public class SwitchPresetCommand extends SequentialCommandGroup
{
    public static SwitchPresetCommand defaultToStow(boolean keepAlive)
    {
        SwitchPresetCommand preset = new SwitchPresetCommand();
        preset.addCommands(
            new InstantCommand(preset::initDefaultToStow),
            new MovePresetCommand(() -> preset.posPoints[0]), // Wrist first
            new MovePresetCommand(() -> preset.posPoints[1]), // Arm second
            new MovePresetCommand(() -> preset.posPoints[2])  // Then stow position.
        );
        if (keepAlive) preset.addCommands(new ForeverCommand());
        return preset;
    }
    public static SwitchPresetCommand stowToDefault(CarriagePreset newPreset, boolean keepAlive)
    {
        SwitchPresetCommand preset = new SwitchPresetCommand();
        preset.addCommands(
            new InstantCommand(() -> preset.initStowToDefault(newPreset)),
            new MovePresetCommand(() -> preset.posPoints[0]), // Elevator first
            new MovePresetCommand(() -> preset.posPoints[1]), // Arm second
            new MovePresetCommand(() -> preset.posPoints[2])  // Wrist third
        );
        if (keepAlive) preset.addCommands(new ForeverCommand());
        return preset;
    }
    public static SwitchPresetCommand loadToStow(boolean keepAlive)
    {
        SwitchPresetCommand preset = new SwitchPresetCommand();
        preset.addCommands(
            new InstantCommand(preset::initLoadToStow),
            new MovePresetCommand(() -> preset.posPoints[0]), // Elevator first
            new MovePresetCommand(() -> preset.posPoints[1])  // Arm and wrist second
        );
        if (keepAlive) preset.addCommands(new ForeverCommand());
        return preset;
    }
    public static SwitchPresetCommand stowToLoad(boolean keepAlive)
    {
        SwitchPresetCommand preset = new SwitchPresetCommand();
        preset.addCommands(
            new InstantCommand(preset::initStowToLoad),
            new MovePresetCommand(() -> preset.posPoints[0]), // Arm and wrist first
            new MovePresetCommand(() -> preset.posPoints[1])  // Elevator second
        );
        if (keepAlive) preset.addCommands(new ForeverCommand());
        return preset;
    }

    private final ElevatorCarriageSubsystem mElevatorCarriage;
    private final IntakeSubsystem mIntake;

    private CarriagePreset[] posPoints;

    private SwitchPresetCommand()
    {
        mElevatorCarriage = ElevatorCarriageSubsystem.kInstance;
        mIntake = IntakeSubsystem.kInstance;
        addRequirements(mElevatorCarriage);
    }

    private void initDefaultToStow()
    {
        // From a position to the stow position, following the standard routine.
        // First move wrist, then arm, then elevator.
        CarriagePreset stowPosition = mIntake.hasAlgae() ? CarriagePreset.kStowAlgae : CarriagePreset.kStowCoral;

        // Determine the current position as a preset.
        CarriagePreset activePos = new CarriagePreset(
            mElevatorCarriage.getArmAngle(),
            mElevatorCarriage.getWristAngle(),
            mElevatorCarriage.getElevatorHeight() - ElevatorCarriageSubsystem.initialHeight
        );

        CarriagePreset wristPos = activePos.withWristPreset(stowPosition);
        CarriagePreset armPos = wristPos.withArmPreset(stowPosition);
        CarriagePreset elevatorPos = armPos.withElevatorPreset(stowPosition);
        posPoints = new CarriagePreset[]
        {
            wristPos,   // Wrist first
            armPos,     // Arm second
            elevatorPos // Elevator last
        };
    }
    private void initStowToDefault(CarriagePreset preset)
    {
        // From the stow position to another position, following the standard routine.
        // Move elevator first, then arm, then wrist.
        CarriagePreset stowPosition = mIntake.hasAlgae() ? CarriagePreset.kStowAlgae : CarriagePreset.kStowCoral;

        CarriagePreset elevatorPos = stowPosition.withElevatorPreset(preset);
        CarriagePreset armPos = elevatorPos.withArmPreset(preset);
        CarriagePreset wristPos = armPos.withWristPreset(preset);
        posPoints = new CarriagePreset[]
        {
            elevatorPos,
            armPos,
            wristPos
        };
    }
    private void initLoadToStow()
    {
        // From the loading position, move to the stow position.
        // Move the elevator first, then the arm & wrist simultaneously.
        CarriagePreset stowPosition = mIntake.hasAlgae() ? CarriagePreset.kStowAlgae : CarriagePreset.kStowCoral;
        CarriagePreset loadPosition = CarriagePreset.kCoralLoad; // Assuming we're here already.

        CarriagePreset elevatorPos = loadPosition.withElevatorPreset(stowPosition);
        CarriagePreset wristArmPos = elevatorPos.withArmPreset(stowPosition).withWristPreset(stowPosition);
        posPoints = new CarriagePreset[]
        {
            elevatorPos, // Elevator first
            wristArmPos  // Wrist and arm second
        };
    }
    private void initStowToLoad()
    {
        // From the stow position, move the the loading position.
        // Move the arm & wrist first (simultaneously), then move the elevator.
        CarriagePreset loadPosition = CarriagePreset.kCoralLoad;
        CarriagePreset stowPosition = mIntake.hasAlgae() ? CarriagePreset.kStowAlgae : CarriagePreset.kStowCoral; // Assuming we're here already.

        CarriagePreset wristArmPos = stowPosition.withArmPreset(loadPosition).withWristPreset(loadPosition);
        CarriagePreset elevatorPos = wristArmPos.withElevatorPreset(loadPosition);
        posPoints = new CarriagePreset[]
        {
            wristArmPos, // Wrist and arm first
            elevatorPos  // Elevator second
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
            // Tolerances for the arm, wrist, and elevator.
            // This is not the point at which they stop moving,
            // but the point at which it is okay to proceed to the next step.
            // We can be pretty generous here.
            final double armDelta = 10,
                         wristDelta = 10,
                         elevatorDelta = 1;
            
            return Math.abs(mElevatorCarriage.getArmAngle() - activePreset.kArmPreset) <= armDelta &&
                   Math.abs(mElevatorCarriage.getWristAngle() - activePreset.kWristPreset) <= wristDelta &&
                   Math.abs(mElevatorCarriage.getElevatorHeight() - activePreset.kElevatorPreset) <= elevatorDelta;
        }
    }
}
