package frc.robot.commands;

import java.util.Optional;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class VisionAlignCommand extends Command {
    private final SwerveSubsystem mSwerveSubsystem;
    private final VisionSubsystem mVisionSubsystem;
    private final Translation2d mOffset;
    private final AprilTagFieldLayout mLayout;

    private final PIDController mLateralPidController;

    public static final double kP= 1, kI = 0, kD = 0;

    private Transform2d mError;

    public VisionAlignCommand(
        SwerveSubsystem pSwerveSubsystem,
        VisionSubsystem pVisionSubsystem,
        Translation2d pOffset
    )
    {
        mSwerveSubsystem = pSwerveSubsystem;
        mVisionSubsystem = pVisionSubsystem;
        mOffset = pOffset;

        mLayout = mVisionSubsystem.getTagLayout();
        mLateralPidController = new PIDController(kP, kI, kD);

        addRequirements(pSwerveSubsystem, pVisionSubsystem);
    }

    private Optional<Translation2d> calculateError()
    {
        if (mVisionSubsystem.getTargetInView().isPresent())
        {
            PhotonTrackedTarget target = mVisionSubsystem.getTargetInView().get();
            Pose2d curPose = mVisionSubsystem.getPose();
            Pose2d targetPose = mLayout.getTagPose(target.getFiducialId()).get().toPose2d();
            
            return Optional.of(targetPose.minus(curPose).getTranslation());
        }
        return Optional.empty();
    }
    
    @Override
    public void execute() {
        if (calculateError().isPresent())
        {
            Translation2d error = calculateError().get();
            double xValue = mLateralPidController.calculate(error.getX(), 0);
            double yValue = mLateralPidController.calculate(error.getY(), 0);
            mSwerveSubsystem.driveTranslation(new Translation2d(xValue, yValue));
        }
    }

    @Override
    public boolean isFinished() {
        
        return false;
    }
}


