package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CarriageSubsystem;

public class MoveCarriageToPresetCommand extends Command
{
    public static final double kArmTolerence = 5.0;
    public static final double kWristTolerance = 5.0;

    private final CarriageSubsystem mCarriageSubsystem;

    private final Double armPreset;
    private final Double wristPreset;

    public MoveCarriageToPresetCommand(CarriageSubsystem carriageSubsystem, Double armPreset, Double wristPreset)
    {
        this.armPreset = armPreset;
        this.wristPreset = wristPreset;

        mCarriageSubsystem = carriageSubsystem;
        addRequirements(mCarriageSubsystem);
    }

    @Override
    public void initialize() 
    {
        mCarriageSubsystem.setArmPreset(armPreset);
        mCarriageSubsystem.setWristPreset(wristPreset);
    }

    @Override
    public void end(boolean stopping) 
    {
        mCarriageSubsystem.stop();
    }

    @Override
    public boolean isFinished() 
    {
        double armDist = mCarriageSubsystem.getArmAngle() - mCarriageSubsystem.getArmSetpoint();
        double wristDist = mCarriageSubsystem.getWristAngle() - mCarriageSubsystem.getWristSetpoint();

        return Math.abs(armDist) < kArmTolerence && Math.abs(wristDist) < kWristTolerance;
    }
}
