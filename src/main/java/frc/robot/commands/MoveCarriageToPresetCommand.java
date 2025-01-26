package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.objectmodels.IntakePresets;
import frc.robot.subsystems.CarriageSubsystem;

public class MoveCarriageToPresetCommand extends Command
{
    private final CarriageSubsystem mCarriageSubsystem;

    private final IntakePresets preset;

    public MoveCarriageToPresetCommand(CarriageSubsystem carriageSubsystem, IntakePresets preset)
    {

        this.preset = preset;

        mCarriageSubsystem = carriageSubsystem;

        addRequirements(mCarriageSubsystem);
    }

    @Override
    public void initialize() 
    {
        mCarriageSubsystem.setActiveArmPreset(preset);

        mCarriageSubsystem.setActiveWristPreset(preset);
    }

    @Override
    public void end(boolean stopping) 
    {
        mCarriageSubsystem.stopArm();
        mCarriageSubsystem.stopWrist();
    }

    @Override
    public boolean isFinished() 
    {
        boolean armAtTarget = mCarriageSubsystem.getArmCurrentPosition() == targetArmPosition;

        boolean wristAtTarget = mCarriageSubsystem.getWristCurrentPosition() == targetWristPosition;

        return armAtTarget && wristAtTarget;
    }
}