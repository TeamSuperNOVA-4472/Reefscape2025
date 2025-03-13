package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.objectmodels.CarriagePreset;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorCarriageSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Moves the carriage to a stow position where the elevator can move freely.
 * Supports algae, it will choose a different preset if algae is detected.
 */
public class moveOutOfDangerZone extends Command
{
    IntakeSubsystem mIntakeSubsystem;
    ElevatorCarriageSubsystem mElevatorCarriageSubsystem;

    double newWristTarget;
    double newArmTarget;

    public moveOutOfDangerZone()
    {
        mElevatorCarriageSubsystem = ElevatorCarriageSubsystem.kInstance;
        mIntakeSubsystem = IntakeSubsystem.instance();
        addRequirements(mElevatorCarriageSubsystem, mIntakeSubsystem);
    }

    @Override
    public void initialize()
    {
        // If the intake has an algae, we need to go to a different spot to be "safe."
        if (mIntakeSubsystem.hasAlgae()) mElevatorCarriageSubsystem.setPreset(CarriagePreset.kStowAlgae);
        else mElevatorCarriageSubsystem.setPreset(CarriagePreset.kStowCoral);
    }

    @Override
    public boolean isFinished()
    {
        double armDist = mElevatorCarriageSubsystem.getArmAngle() - mElevatorCarriageSubsystem.getArmSetpoint();
        double wristDist = mElevatorCarriageSubsystem.getAbsoluteWristAngle() - mElevatorCarriageSubsystem.getWristSetpoint();

        return Math.abs(armDist) < MoveCarriageToPresetCommand.kArmTolerence && Math.abs(wristDist) < MoveCarriageToPresetCommand.kWristTolerance;
    }
}
