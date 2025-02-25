package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class MoveOutOfDangerZone extends Command
{
    IntakeSubsystem mIntakeSubsystem;
    CarriageSubsystem mCarriageSubsystem;

    double newWristTarget;
    double newArmTarget;

    public MoveOutOfDangerZone(CarriageSubsystem pCarriageSubsystem, IntakeSubsystem pIntakeSubsystem)
    {
        mCarriageSubsystem = pCarriageSubsystem;
        mIntakeSubsystem = pIntakeSubsystem;
        addRequirements(mCarriageSubsystem);
    }

    @Override
    public void initialize()
    {
        newWristTarget = mCarriageSubsystem.getWristSetpoint();
        newArmTarget = mCarriageSubsystem.getArmSetpoint();
        if (mIntakeSubsystem.hasAlgae())
        {
            newArmTarget = CarriageSubsystem.armAlgaeStow;
            newWristTarget = CarriageSubsystem.wristAlgaeStow;
        } 
        else if (mCarriageSubsystem.getCarriageTargetX() < -5)
        {
            newWristTarget = CarriageSubsystem.wristPresetMoving;
            newArmTarget = CarriageSubsystem.armPresetMoving;
        }
        mCarriageSubsystem.setArmPreset(newArmTarget);
        mCarriageSubsystem.setWristPreset(newWristTarget);
    }

    @Override
    public boolean isFinished()
    {
        double armDist = mCarriageSubsystem.getArmAngle() - mCarriageSubsystem.getArmSetpoint();
        double wristDist = mCarriageSubsystem.getWristAngle() - mCarriageSubsystem.getWristSetpoint();

        return Math.abs(armDist) < MoveCarriageToPresetCommand.kArmTolerence && Math.abs(wristDist) < MoveCarriageToPresetCommand.kWristTolerance;
    }
}
