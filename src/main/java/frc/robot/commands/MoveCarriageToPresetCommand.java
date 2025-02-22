package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.objectmodels.IntakePreset;
import frc.robot.subsystems.CarriageSubsystem;

public class MoveCarriageToPresetCommand extends Command
{
    public static final double kArmTolerence = 5.0;
    public static final double kWristTolerance = 5.0;

    private final CarriageSubsystem mCarriageSubsystem;

    private final IntakePreset preset;

    public MoveCarriageToPresetCommand(CarriageSubsystem carriageSubsystem, IntakePreset preset)
    {
        this.preset = preset;

        mCarriageSubsystem = carriageSubsystem;
        addRequirements(mCarriageSubsystem);
    }

    @Override
    public void initialize() 
    {
        mCarriageSubsystem.setArmPreset(preset.armPreset);
        mCarriageSubsystem.setWristPreset(preset.wristPreset);
    }

    @Override
    public boolean isFinished() 
    {
        double armDist = mCarriageSubsystem.getArmAngle() - mCarriageSubsystem.getArmSetpoint();
        double wristDist = mCarriageSubsystem.getWristAngle() - mCarriageSubsystem.getWristSetpoint();

        return Math.abs(armDist) < kArmTolerence && Math.abs(wristDist) < kWristTolerance;
    }
}
