package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.objectmodels.CarriagePreset;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Moves the carriage to a stow position where the elevator can move freely.
 * Supports algae, it will choose a different preset if algae is detected.
 */
public class moveOutOfDangerZone extends Command
{
    IntakeSubsystem mIntakeSubsystem;
    CarriageSubsystem mCarriageSubsystem;

    double newWristTarget;
    double newArmTarget;

    public moveOutOfDangerZone()
    {
        mCarriageSubsystem = CarriageSubsystem.instance();
        mIntakeSubsystem = IntakeSubsystem.instance();
        addRequirements(mCarriageSubsystem, mIntakeSubsystem);
    }

    @Override
    public void initialize()
    {
        // If the intake has an algae, we need to go to a different spot to be "safe."
        if (mIntakeSubsystem.hasAlgae()) mCarriageSubsystem.setPreset(CarriagePreset.kStowAlgae);
        else mCarriageSubsystem.setPreset(CarriagePreset.kStowCoral);
    }

    @Override
    public boolean isFinished()
    {
        double armDist = mCarriageSubsystem.getArmAngle() - mCarriageSubsystem.getArmSetpoint();
        double wristDist = mCarriageSubsystem.getWristAngle() - mCarriageSubsystem.getWristSetpoint();

        return Math.abs(armDist) < MoveCarriageToPresetCommand.kArmTolerence && Math.abs(wristDist) < MoveCarriageToPresetCommand.kWristTolerance;
    }
}
