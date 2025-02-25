package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CarriageSubsystem;

public class moveOutOfDangerZone extends Command
{

    CarriageSubsystem mCarriageSubsystem;

    double newWristTarget;
    double newArmTarget;

    public moveOutOfDangerZone(CarriageSubsystem pCarriageSubsystem)
    {
        mCarriageSubsystem = pCarriageSubsystem;

        newWristTarget = mCarriageSubsystem.getWristSetpoint();
        newArmTarget = mCarriageSubsystem.getArmSetpoint();
    }

    @Override
    public void initialize()
    {
        if (mCarriageSubsystem.getCarriageTargetX() < -5)
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
