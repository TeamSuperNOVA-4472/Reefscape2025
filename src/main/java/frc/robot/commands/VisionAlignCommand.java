package frc.robot.commands;

import java.lang.StackWalker.Option;
import java.util.Optional;

import org.opencv.photo.Photo;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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

    private DriveDistanceAndHeading mDrive;
    private Optional<PhotonTrackedTarget> mLastTarget;
    private Transform2d mError;
    private boolean mMoving;

    // Creates new command with swerve, vision, and offset
    public VisionAlignCommand(
        SwerveSubsystem pSwerveSubsystem,
        VisionSubsystem pVisionSubsystem,
        Translation2d pOffset
    )
    {
        mSwerveSubsystem = pSwerveSubsystem;
        mVisionSubsystem = pVisionSubsystem;
        mOffset = pOffset;

        mMoving = false;

        mLayout = mVisionSubsystem.getTagLayout();

        addRequirements(pSwerveSubsystem, pVisionSubsystem);
    }

    // Updates the target
    private Optional<PhotonTrackedTarget> updateTarget()
    {
        if (mVisionSubsystem.getTargetInView().isPresent())
        {
            return Optional.of(mVisionSubsystem.getTargetInView().get());
        }
        return Optional.empty();
    }

    private Optional<Pose3d> getTagPose(PhotonTrackedTarget target)
    {
        return mLayout.getTagPose(target.getFiducialId());
    }

    // Returns a translation that is the difference between the robot's current pose and desired pose
    private Translation2d getTranslation(PhotonTrackedTarget target)
    {
        Pose3d curPose = mVisionSubsystem.getPose();
        Pose3d targetPose = getTagPose(target).get();
            
        return targetPose.minus(curPose).getTranslation().toTranslation2d();
    }

    // Updates the target
    @Override
    public void initialize() {
        mLastTarget = updateTarget();
    }

    
    // Checks if the target is empty. If it is, updates the target. If not, it checks if the module is
    // already moving, and if it isn't, tells the swerve to perform the calculated translation.
    @Override
    public void execute() {
        
        if (mLastTarget.isEmpty())
        {
            mLastTarget = updateTarget();
        }
        else if (mDrive==null||!mDrive.isScheduled())
        {
            mMoving = true;
            mDrive = new DriveDistanceAndHeading(
                mSwerveSubsystem, 
                getTranslation(mLastTarget.get()),
                getTagPose(mLastTarget.get()).get().getRotation().getAngle() - mVisionSubsystem.getPose().getRotation().getAngle()
            );
            mDrive.schedule();
        }
    }

    @Override
    public void end(boolean interrupted) {
        if(mDrive!=null)
        {
            mDrive.cancel();
        }
    }

    //TODO unfinished. needs to exist when appropriate
    @Override
    public boolean isFinished() {

        return false;
    }
}


