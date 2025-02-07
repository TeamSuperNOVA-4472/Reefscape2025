package frc.robot.commands;

import java.util.Optional;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class VisionAlignCommand extends Command {

    public static final Translation2d kLeft = new Translation2d(-1.0, 0.25);
    public static final Translation2d kRight = new Translation2d(-1.0, -0.25);
    public static final Translation2d kNothing = new Translation2d(-1.0, 0);

    private final SwerveSubsystem mSwerveSubsystem;
    private final VisionSubsystem mVisionSubsystem;
    private final AprilTagFieldLayout mLayout;

    private SequentialCommandGroup mSequence;
    private Translation2d mOffset;
    private Optional<PhotonTrackedTarget> mLastTarget;

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

        mLayout = mVisionSubsystem.getTagLayout();

        addRequirements(pSwerveSubsystem, pVisionSubsystem);
    }

    // METHOD - updates target in sight if visible
    private Optional<PhotonTrackedTarget> updateTarget()
    {
        if (mVisionSubsystem.getTargetInView().isPresent())
        {
            return Optional.of(mVisionSubsystem.getTargetInView().get());
        }
        return Optional.empty();
    }

    // METHOD - get destination pose
    private Optional<Pose3d> getTagPose(PhotonTrackedTarget target)
    {
        return mLayout.getTagPose(target.getFiducialId());
    }

    private Pose3d transformToPose3d(Transform3d transform)
    {
        return new Pose3d(transform.getTranslation(), transform.getRotation());
    }

    private double normalizeDegree(double deg, double offset)
    {
        return (deg+360+offset)%360;
    }

    private double getDegreesFromPose(Pose3d pose)
    {
        return pose.getRotation().toRotation2d().getDegrees();
    }
    
    private Pose3d getTransformToCameraAsPose()
    {
        return transformToPose3d(mVisionSubsystem.getCameraToTarget().get());
    }

    private Pose3d getDestinationAsPose()
    {
        return getTagPose(mLastTarget.get()).get();
    }
    
    private double getNormalizedDegreesBetweenPoses(Pose3d pose, Pose3d subtractedPose)
    {

        double normalizedDeg = normalizeDegree(getDegreesFromPose(pose), 180);
        double normalizedSubDeg = normalizeDegree(getDegreesFromPose(subtractedPose), 0);

        return normalizedDeg - normalizedSubDeg;
    }

    public void setOffset(Translation2d newOffset)
    {
        mOffset = newOffset;
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
        else
        {
            Pose3d dest = getDestinationAsPose();

            double difference = getNormalizedDegreesBetweenPoses(dest, mVisionSubsystem.getPose());

            mSequence = new SequentialCommandGroup(
                new DriveDistanceAndHeading(
                    mSwerveSubsystem,
                    () -> {
                        return new Pose2d(
                            0,
                            0,
                            Rotation2d.fromDegrees(difference)
                        );
                    }
                ),
                new DriveDistanceAndHeading(
                    mSwerveSubsystem,
                    () -> {
                        Pose3d curr = getTransformToCameraAsPose();
                        return new Pose2d(
                            curr.getX() + mOffset.getX(),
                            curr.getY() + mOffset.getY(),
                            Rotation2d.fromDegrees(0)
                        );
                    }
                )
            );
            mSequence.schedule();
        }
    }

    //TODO doesn't work
    @Override
    public void end(boolean interrupted) {
        if(mSequence != null)
        {
            mSequence.cancel();
        }
    }

    //TODO unfinished. needs to exist when appropriate
    @Override
    public boolean isFinished() {
        return mSequence != null && mSequence.isFinished() ? true : false;
    }
}


