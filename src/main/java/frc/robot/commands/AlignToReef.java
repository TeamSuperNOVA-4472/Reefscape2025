package frc.robot.commands;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import com.ctre.phoenix.CANifier.PinValues;
import com.ctre.phoenix6.swerve.jni.SwerveJNI.DriveState;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AlignToReef extends SequentialCommandGroup {
    public static enum EndTarget {
        FAR_LEFT,
        FAR_MIDDLE,
        FAR_RIGHT,
        NEAR_LEFT,
        NEAR_MIDDLE,
        NEAR_RIGHT
    }

    // Define member variables.
    private final SwerveSubsystem mSwerveSubsystem;
    private final VisionSubsystem mVisionSubsystem;
    private final EndTarget mEndTarget;

    private final Transform2d wayPointTransform = new Transform2d(1.5, 0, Rotation2d.k180deg);
    private final Transform2d endingPointTransform = new Transform2d(1.5, 0, Rotation2d.k180deg);

    public AlignToReef(
        SwerveSubsystem pSwerveSubsystem, 
        VisionSubsystem pVisionSubsystem, 
        EndTarget pEndTarget)
    {
        mSwerveSubsystem = pSwerveSubsystem;
        mVisionSubsystem = pVisionSubsystem;
        mEndTarget = pEndTarget;

        super.addCommands(
            pathFindToPlace(), 
            new DeferredCommand(() ->
                new CloseUpOnReef(mSwerveSubsystem, new Pose2d(0, 0.2, new Rotation2d())),
                Set.of(mSwerveSubsystem)
            )
        );
    }

    private ArrayList<Pose2d> createPoses()
    {
        ArrayList<Pose2d> poses = new ArrayList<Pose2d>();
        int index = isBlueAlliance() ? 17 : 6;

        for (int i = 0; i < 6; i++)
        {
            Pose2d visionPose = mVisionSubsystem.getTagLayout().getTagPose(i+index).get().toPose2d();
            Pose2d fixedPose = visionPose.plus(wayPointTransform);

            poses.add(fixedPose);
        }

        return poses;
    }

    private Pose2d findTargetPose(EndTarget target)
    {
        int[] tags = new int[6];
        if (isBlueAlliance())
        {
            tags[0] = 19;
            tags[1] = 18;
            tags[2] = 17;
            tags[3] = 20;
            tags[4] = 21;
            tags[5] = 22;
        }
        else
        {
            tags[0] = 6;
            tags[1] = 7;
            tags[2] = 8;
            tags[3] = 11;
            tags[4] = 10;
            tags[5] = 9;
        }

        int targetId = 0;

        switch (target)
        {
            case NEAR_LEFT:
                targetId = tags[0];
                break;
            case NEAR_MIDDLE:
                targetId = tags[1];
                break;
            case NEAR_RIGHT:
                targetId = tags[2];
                break;
            case FAR_LEFT:
                targetId = tags[3];
                break;
            case FAR_MIDDLE:
                targetId = tags[4];
                break;
            case FAR_RIGHT:
                targetId = tags[5];
                break;
        }

        return mVisionSubsystem.getTagLayout().getTagPose(targetId).get().toPose2d();
    }

    private Command pathFindToPlace()
    {
        // Create a list of poses
        ArrayList<Pose2d> poses = createPoses();

        Pose2d currPose = mSwerveSubsystem.getPose();
        Pose2d endingPose = findTargetPose(mEndTarget);
        
        ArrayList<Pose2d> pathList = getMinPath(poses, endingPose.plus(wayPointTransform));

        endingPose = endingPose.plus(endingPointTransform);

        pathList.add(0, currPose);
        pathList.add(endingPose);

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(pathList.toArray(new Pose2d[0]));

        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            new PathConstraints(2, 2, Units.degreesToRadians(360), Units.degreesToRadians(540)),
            null,
            new GoalEndState(2, endingPose.getRotation())
        );

        path.preventFlipping = true;

        return AutoBuilder.followPath(path);
    }

    private ArrayList<Pose2d> getMinPath(ArrayList<Pose2d> poses, Pose2d target)
    {
        ArrayList<Pose2d> reversePoses = new ArrayList<Pose2d>();
        reversePoses.addAll(poses);
        Collections.reverse(reversePoses);
        
        ArrayList<Pose2d> path = new ArrayList<Pose2d>();
        ArrayList<Pose2d> altPath = new ArrayList<Pose2d>();

        Pose2d nearestPose = mSwerveSubsystem.getPose().nearest(poses);

        int fwdIndex = poses.indexOf(nearestPose);
        int revIndex = reversePoses.indexOf(nearestPose);

        for (int i = 0; i < 3; i++)
        {
            Pose2d curFwdPose = poses.get( (fwdIndex + i) % 6);
            Pose2d curRevPose = reversePoses.get( (revIndex + i) % 6);

            if (curFwdPose.equals(target)) { return path; }
            if (curRevPose.equals(target)) { return altPath; }
            
            path.add(new Pose2d(curFwdPose.getTranslation(), curFwdPose.getRotation().rotateBy(Rotation2d.kCW_90deg)));
            altPath.add(new Pose2d(curRevPose.getTranslation(), curRevPose.getRotation().rotateBy(Rotation2d.kCCW_90deg)));
        }
        
        return path;
    }

    private boolean isBlueAlliance()
    {
        // Returns true if the robot is connected to the FMS and on the blue alliance
        return DriverStation.getAlliance().get().equals(Alliance.Blue);
    }
}
