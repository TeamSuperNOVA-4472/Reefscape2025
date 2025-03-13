package frc.robot.commands;

import java.util.Optional;

import org.photonvision.proto.Photon;
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

public class VisionAlignCommandTemp extends SequentialCommandGroup {

    public static final Translation2d kLeft = new Translation2d(-1.0, 0.25);
    public static final Translation2d kRight = new Translation2d(-1.0, -0.25);
    public static final Translation2d kNothing = new Translation2d(-0.5, 0);

    private final SwerveSubsystem mSwerveSubsystem;
    private final VisionSubsystem mVisionSubsystem;
    private final AprilTagFieldLayout mLayout;
    private final PhotonTrackedTarget mTarget;

    private SequentialCommandGroup mSequence;
    private DriveDistanceAndHeading mRotate;
    private DriveDistanceAndHeading mLateral;
    private Translation2d mOffset;

    // Creates new command with swerve, vision, and offset
    public VisionAlignCommandTemp(
        SwerveSubsystem pSwerveSubsystem,
        VisionSubsystem pVisionSubsystem,
        Translation2d pOffset
    )
    {
        mSwerveSubsystem = pSwerveSubsystem;
        mVisionSubsystem = pVisionSubsystem;
        mOffset = pOffset;

        mLayout = mVisionSubsystem.getTagLayout();
        mTarget = mVisionSubsystem.getLastSeenTarget();

        mRotate = new DriveDistanceAndHeading(
            () -> {
                double difference;
                if (mVisionSubsystem.getLastSeenTarget() != null)
                {
                    Pose3d dest = getDestinationAsPose();
                    difference = getNormalizedDegreesBetweenPoses(dest, mVisionSubsystem.getPose());
                }
                else
                {
                    System.out.println("[ALIGN] !!! Trying to align with vision when a target does not exist!");
                    difference = 0; // No vision target, don't move the robot.
                }

                return new Pose2d(
                    0,
                    0,
                    Rotation2d.fromDegrees(difference)
                );
            }
        );

        mLateral = new DriveDistanceAndHeading(
            () -> {
                PhotonTrackedTarget lastSeenTarget = mVisionSubsystem.getLastSeenTarget();
                if (lastSeenTarget != null)
                {
                    Pose3d curr = getTransformToCameraAsPose(lastSeenTarget);
                    return new Pose2d(
                        curr.getX() + mOffset.getX(),
                        curr.getY() + mOffset.getY(),
                        Rotation2d.fromDegrees(0)
                    );
                }
                else
                {
                    // No vision target, don't move the robot.
                    return Pose2d.kZero;
                }
            }
        );
    
        addCommands(mRotate, mLateral);
        //addRequirements(pSwerveSubsystem, pVisionSubsystem);
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
    private Pose3d getTagPose(PhotonTrackedTarget target)
    {
        return mLayout.getTagPose(target.getFiducialId()).get();
    }

    private Pose3d transformToPose3d(Transform3d transform)
    {
        return new Pose3d(transform.getTranslation(), transform.getRotation());
    }

    // Normalizes the degree to 0-360
    private double normalizeDegree(double deg, double offset)
    {
        return (deg+360+offset)%360;
    }

    // Gets the rotation in degrees for a Pose3d
    private double getDegreesFromPose(Pose3d pose)
    {
        return pose.getRotation().toRotation2d().getDegrees();
    }
    
    // Returns the Pose3d from a transform
    private Pose3d getTransformToCameraAsPose(PhotonTrackedTarget target)
    {
        return transformToPose3d(mVisionSubsystem.getCameraToTarget(target));
    }

    // Returns the Pose3d of the destination
    private Pose3d getDestinationAsPose()
    {
        return getTagPose(mVisionSubsystem.getLastSeenTarget());
    }
    
    // Returns the normalized difference in degrees between two Pose3ds
    private double getNormalizedDegreesBetweenPoses(Pose3d pose, Pose3d subtractedPose)
    {

        double normalizedDeg = normalizeDegree(getDegreesFromPose(pose), 0);
        double normalizedSubDeg = normalizeDegree(getDegreesFromPose(subtractedPose), 0);

        return normalizedDeg - normalizedSubDeg;
    }

    // Sets the offset
    public void setOffset(Translation2d newOffset)
    {
        mOffset = newOffset;
    }
}




