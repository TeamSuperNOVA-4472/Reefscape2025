package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.objectmodels.IntakePresets;
import frc.robot.subsystems.CarriageSubsystem;

public class MoveCarriageToPresetCommand extends Command
{
    public static final double kArmTolerence = 1.0;
    public static final double kWristTolerance = 1.0;

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
        mCarriageSubsystem.setActivePreset(preset);
    }

    @Override
    public void end(boolean stopping) 
    {
        mCarriageSubsystem.stop();
    }

    @Override
    public boolean isFinished() 
    {
        double armDist = mCarriageSubsystem.getArmCurrentPosition() - mCarriageSubsystem.getArmSetpoint();
        double wristDist = mCarriageSubsystem.getWristCurrentPosition() - mCarriageSubsystem.getWristSetpoint();

        return Math.abs(armDist) < kArmTolerence && Math.abs(wristDist) < kWristTolerance;
    }
}
