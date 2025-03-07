package frc.robot.commands.Vision;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import com.ctre.phoenix.CANifier.PinValues;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AlignToReef extends SequentialCommandGroup {
    // Define member variables.
    private final SwerveSubsystem mSwerveSubsystem;
    private final VisionSubsystem mVisionSubsystem;
    private final int mTargetId;

    public AlignToReef(
        SwerveSubsystem pSwerveSubsystem, 
        VisionSubsystem pVisionSubsystem, 
        int pTargetId)
    {
        mSwerveSubsystem = pSwerveSubsystem;
        mVisionSubsystem = pVisionSubsystem;
        mTargetId = pTargetId;

        super.addCommands(pathFindToPlace());
    }

    private Command pathFindToPlace()
    {
        // Create a list of poses
        ArrayList<Pose2d> poses = new ArrayList<Pose2d>();

        Pose2d currPose = mSwerveSubsystem.getPose();
        Pose2d aprilTagPose = mVisionSubsystem.getTagLayout().getTagPose(mTargetId).get().toPose2d().transformBy(new Transform2d(2, 0, Rotation2d.k180deg));

        // TODO: Make this work for blue alliances as well 
        for (int i = 0; i < 6; i++)
        {
            Pose2d visionPose = mVisionSubsystem.getTagLayout().getTagPose(i+6).get().toPose2d();
            Pose2d fixedPose = visionPose.plus(new Transform2d(2, 0, Rotation2d.k180deg));

            poses.add(fixedPose);
    
        }
        
        Pose2d startingPose = currPose;
        Pose2d endingPose = aprilTagPose;
        ArrayList<Pose2d> pathList = getMinPath(poses, aprilTagPose);
        pathList.add(0, startingPose);
        pathList.add(endingPose.transformBy(new Transform2d(1, 0, new Rotation2d())));
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(pathList.toArray(new Pose2d[0]));

        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            new PathConstraints(4.5, 4.5, Units.degreesToRadians(360), Units.degreesToRadians(540)),
            null,
            new GoalEndState(0, new Rotation2d())
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
}
