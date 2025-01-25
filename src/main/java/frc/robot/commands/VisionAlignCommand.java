package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class VisionAlignCommand extends Command {
    private final SwerveSubsystem mSwerveSubsystem;
    private final VisionSubsystem mVisionSubsystem;
    private final Translation2d mOffset;

    public VisionAlignCommand(
        SwerveSubsystem pSwerveSubsystem,
        VisionSubsystem pVisionSubsystem,
        Translation2d pOffset
    )
    {
        mSwerveSubsystem = pSwerveSubsystem;
        mVisionSubsystem = pVisionSubsystem;
        mOffset = pOffset;
    }
    
    @Override
    public void execute() {
        
        
    }

    @Override
    public boolean isFinished() {
        
        return false;
    }
}


