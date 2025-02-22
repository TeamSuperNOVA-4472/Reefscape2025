package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CarriageSubsystem;

public class moveOutOfDangerZone extends Command
{

    CarriageSubsystem mCarriageSubsystem;

    double newWristTarget;
    double newArmTarget; 

    MoveCarriageToPresetCommand move;

    public moveOutOfDangerZone(CarriageSubsystem pCarriageSubsystem)
    {
        mCarriageSubsystem = pCarriageSubsystem;

        newWristTarget = mCarriageSubsystem.getWristSetpoint();
        newArmTarget = mCarriageSubsystem.getArmSetpoint();

        addRequirements(mCarriageSubsystem);
    }

    @Override
    public void initialize()
    {
        if (mCarriageSubsystem.getCarriageTargetX() < -5)
        {
            newWristTarget = CarriageSubsystem.wristPresetMoving;

            newArmTarget = CarriageSubsystem.armPresetMoving;
        }

        move = new MoveCarriageToPresetCommand(mCarriageSubsystem, newArmTarget, newWristTarget);
    }

    @Override
    public boolean isFinished()
    {
        return move.isFinished();
    }
}
