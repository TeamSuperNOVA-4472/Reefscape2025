package frc.robot.commands;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.objectmodels.VisionPoses;
import frc.robot.objectmodels.ReefEndTarget;
import frc.robot.objectmodels.VisionDirection;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class VisionAlign {
    // Member variables
    private final SwerveSubsystem mSwerveSubsystem;
    private final VisionSubsystem mVisionSubsystem;

    private final ProfiledPIDController mXPIDController;
    private final ProfiledPIDController mYPIDController;
    private final ProfiledPIDController mRotationPIDController;
    
    // Constants
    private final Transform2d kWayPointTransform = new Transform2d(1.5, 0, Rotation2d.k180deg); // Controls radius of reef-avoiding
    private final Transform2d kReefTransform = new Transform2d(1.5, 0, Rotation2d.k180deg); // Position at reef
    private final Transform2d kMatchLoadingTransform = new Transform2d(1, 0, Rotation2d.k180deg); // Position at match loading station
    private final Transform2d kProcessorTransform = new Transform2d(1, 0, Rotation2d.k180deg); // Position at processor

    private final Transform2d kLeftCoralTransform = new Transform2d(0.4, -0.25, Rotation2d.k180deg); // Position at left coral stick
    private final Transform2d kRightCoralTransform = new Transform2d(0.4, 0.25, Rotation2d.k180deg); // Position at right coral stick
    private final Transform2d kAlgaeTransform = new Transform2d(0.4, 0, Rotation2d.k180deg); // Position at algae
    private final Transform2d kMatchLoadingApproachTransform = new Transform2d(0.65, 0, new Rotation2d());

    private final double kMaxVelocity = 1.5; // Max velocity for PathPlanner
    private final double kMaxAcceleration = 1.5; // Max acceleration for PathPlanner

    // PID Constants for the Lateral PID Controller
    private final double kLateralP = 3;
    private final double kLateralI = 0;
    private final double kLateralD = 0.2;
    private final double kLateralTolerance = 0.02;

    // PID Constants for the Rotation PID Controller
    private final double kRotationP = 5;
    private final double kRotationI = 0;
    private final double kRotationD = 0;
    private final double kRotationTolerance = 0.02;

    // Gyro input constants
    private final double kMinContinuous = 0;
    private final double kMaxContinuous = 2 * Math.PI;

    /**
     * Creates a new VisionAlign object
     * @param pSwerveSubsystem The swerve subsystem.
     * @param pVisionSubsystem The vision subsystem.
     */
    public VisionAlign(SwerveSubsystem pSwerveSubsystem, VisionSubsystem pVisionSubsystem)
    {
        // Initialize subsystems
        mSwerveSubsystem = pSwerveSubsystem;
        mVisionSubsystem = pVisionSubsystem;

        mXPIDController = new ProfiledPIDController(kLateralP, kLateralI, kLateralD, new Constraints(1.0, 1.0));
        mYPIDController = new ProfiledPIDController(kLateralP, kLateralI, kLateralD, new Constraints(1.0, 1.0));
        mRotationPIDController = new ProfiledPIDController(kRotationP, kRotationI, kRotationD, new Constraints(90, 90));

        mRotationPIDController.enableContinuousInput(kMinContinuous, kMaxContinuous);

        mXPIDController.setTolerance(kLateralTolerance);
        mYPIDController.setTolerance(kLateralTolerance);
        mRotationPIDController.setTolerance(kRotationTolerance);
    }

    /**
     * Pathfinds and aligns to a face on the reef.
     * @param target The enum describing end target from field-oriented position.
     * @param direction The supplier providing an enum describing left, right, or middle PID alignment.
     * @return The sequence of pathfinding and PID alignment.
     */
    private SequentialCommandGroup alignToReef(ReefEndTarget target, Command runLast)
    {
        // Final poses
        Pose2d destination = VisionPoses.getTargetPose(target, mVisionSubsystem); // Pose after PID runs
        Pose2d exitPoint = destination.plus(kReefTransform); // Pose after pathfinding

        try {

            // Creates and returns command
            Command pathFind = pathFindToPlace(exitPoint, kWayPointTransform);
            SequentialCommandGroup alignCommand = new SequentialCommandGroup(pathFind, runLast);

            return alignCommand;

        } catch (Exception ex)
        {
            System.out.println("[ALIGN] Not enough waypoints!");
            return new SequentialCommandGroup(new InstantCommand());
        }
    }

    /**
     * Pathfinds and aligns to a face on the reef.
     * @param target The enum describing end target from field-oriented position.
     * @param leftButton The button that moves the final alignment to the left.
     * @param rightButton The button that moves the final alignment to the right.
     * @return The sequence of pathfinding and PID alignment.
     */
    public SequentialCommandGroup alignToReef(ReefEndTarget target, Supplier<Boolean> leftButton, Supplier<Boolean> rightButton)
    {
        Pose2d destination = VisionPoses.getTargetPose(target, mVisionSubsystem);
        Supplier<VisionDirection> direction = () -> getOffset(leftButton, rightButton);

        Command getClose = getCloseToCommand(destination, direction);

        return alignToReef(target, getClose);
    }

    /**
     * Pathfinds and aligns to a face on the reef.
     * @param target The enum describing end target from field-oriented position.
     * @param direction The enum describing left, right, or middle PID alignment.
     * @return The sequence of pathfinding and PID alignment.
     */
    public SequentialCommandGroup alignToReef(ReefEndTarget target, VisionDirection direction)
    {
        Pose2d destination = VisionPoses.getTargetPose(target, mVisionSubsystem);
        Command getClose = getCloseToCommand(destination, direction);
        
        return alignToReef(target, getClose);
    }

    private SequentialCommandGroup alignToMatchLoadingStation(Pose2d destination)
    {
        destination = destination.plus(kMatchLoadingTransform);
        Pose2d target = destination.nearest(VisionPoses.getReefPoses(kWayPointTransform, mVisionSubsystem));

        Command approach = getCloseToCommand(destination, () -> VisionDirection.MatchLoading);
        
        try {
            return new SequentialCommandGroup(pathFindToPlace(target, destination, kWayPointTransform), approach);
        } catch (Exception e) {
            System.out.println("[ALIGN] Not enough waypoints!");
            return new SequentialCommandGroup(new InstantCommand());
        }
    }

    /**
     * Aligns to the left match loading station from field-oriented view.
     * @return The sequence of pathfinding and PID alignment.
     */
    public SequentialCommandGroup alignToLeftMatchLoadingStation()
    {
        // TODO: add support for right/left trigger movement
        Pose2d destination = VisionPoses.getLeftMatchLoadingStationPose(mVisionSubsystem);
        
        return alignToMatchLoadingStation(destination);
    }

    /**
     * Aligns to the right match loading station from field-oriented view.
     * @return The sequence of pathfinding and PID alignment.
     */
    public SequentialCommandGroup alignToRightMatchLoadingStation()
    {
        // TODO: add support for right/left trigger movement
        Pose2d destination = VisionPoses.getRightMatchLoadingStationPose(mVisionSubsystem);
        
        return alignToMatchLoadingStation(destination);
    }

    /**
     * Aligns to the processor.
     * @return The sequence of pathfinding and PID alignment.
     */
    public SequentialCommandGroup alignToProcessor()
    {
        Pose2d destination = VisionPoses.getProcessorPose(mVisionSubsystem).plus(kProcessorTransform);
        Pose2d target = destination.nearest(VisionPoses.getReefPoses(kWayPointTransform, mVisionSubsystem));
        
        try {
            return new SequentialCommandGroup(pathFindToPlace(target, destination, kWayPointTransform));
        } catch (Exception e) {
            System.out.println("[ALIGN] Not enough waypoints!");
            return new SequentialCommandGroup(new InstantCommand());
        }
    }

    // Gets a supplier with the correct offset for left/right
    private VisionDirection getOffset(Supplier<Boolean> leftButton, Supplier<Boolean> rightButton)
    {
        if (leftButton.get())
        {
            return VisionDirection.LeftCoral;
        }
        else if (rightButton.get())
        {
            return VisionDirection.RightCoral;
        }
        else
        {
            return VisionDirection.MiddleDefault;
        }
    }

    // Calls path find when exit point and destination are the same
    private Command pathFindToPlace(Pose2d target, Transform2d radius)
    {
        return pathFindToPlace(target, target, radius);
    }

    // Path finds to a destination while avoiding the reef
    private Command pathFindToPlace(Pose2d target, Pose2d lastPose, Transform2d radius)
    {
        // Create a list of poses
        ArrayList<Pose2d> poses = VisionPoses.getReefPoses(radius, mVisionSubsystem);

        Pose2d currPose = mSwerveSubsystem.getPose(); // Gets the current position of the drive
        Pose2d endingPose = target; // Gets the exit point position
        
        ArrayList<Pose2d> pathList = getMinPath(poses, endingPose, currPose); // Get the shortest path to the exit point

        pathList.add(0, currPose); // Add the current pose
        pathList.add(lastPose); // Add the last pose

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(pathList.toArray(new Pose2d[0])); // Create waypoints
        
        // Create PathPlanner on-the-fly path with waypoints
        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            new PathConstraints(kMaxVelocity, kMaxAcceleration, Units.degreesToRadians(360), Units.degreesToRadians(540)),
            null,
            new GoalEndState(kMaxVelocity, lastPose.getRotation())
        );

        path.preventFlipping = true;

        return AutoBuilder.followPath(path);
    }

    // Gets the quickest path to the target
    private ArrayList<Pose2d> getMinPath(ArrayList<Pose2d> poses, Pose2d target, Pose2d curPose)
    {
        // Create reverse list
        ArrayList<Pose2d> reversePoses = new ArrayList<Pose2d>();
        reversePoses.addAll(poses);
        Collections.reverse(reversePoses);
        
        // Initialize paths
        ArrayList<Pose2d> path = new ArrayList<Pose2d>();
        ArrayList<Pose2d> altPath = new ArrayList<Pose2d>();

        // Find the nearest pose to the current pose
        Pose2d nearestPose = curPose.nearest(poses);

        // Find starting indexes for each path
        int fwdIndex = poses.indexOf(nearestPose);
        int revIndex = reversePoses.indexOf(nearestPose);

        // Path iterates forward (clockwise) whereas alternative path iterates backward (counter clockwise)
        for (int i = 0; i < 3; i++)
        {
            // Get the current pose
            Pose2d curFwdPose = poses.get( (fwdIndex + i) % 6);
            Pose2d curRevPose = reversePoses.get( (revIndex + i) % 6);

            // Return either path if it has the target
            if (curFwdPose.equals(target)) { return path; }
            if (curRevPose.equals(target)) { return altPath; }
            
            // Add pose to path and adjust angle to face the direction of travel
            path.add(new Pose2d(curFwdPose.getTranslation(), curFwdPose.getRotation().rotateBy(Rotation2d.kCW_90deg)));
            altPath.add(new Pose2d(curRevPose.getTranslation(), curRevPose.getRotation().rotateBy(Rotation2d.kCCW_90deg)));
        }
        
        // If both paths are equally long, return the clockwise one
        return path;
    }

    // Creates the PID command that runs until interrupted for teleop control
    private Command getCloseToCommand(Pose2d destination, Supplier<VisionDirection> direction)
    {
        Command runCommand  =
            new RunCommand(() -> getCloseTo(destination, direction), mSwerveSubsystem, mVisionSubsystem);
        return new SequentialCommandGroup(
            resetProfiledPidCommand(),
            runCommand
        );
    }

    // Creates the PID command that runs until controllers are at setpoint
    private Command getCloseToCommand(Pose2d destination, VisionDirection direction)
    {
        Command command = new RunCommand(() -> getCloseTo(destination, () -> direction), mSwerveSubsystem, mVisionSubsystem)
            .until(() ->
                mXPIDController.atGoal() &&
                mYPIDController.atGoal() &&
                mRotationPIDController.atGoal()
            );

            return new SequentialCommandGroup(
                resetProfiledPidCommand(),
                command);
    }

    private Command resetProfiledPidCommand() {
        return new InstantCommand(() -> {
            Pose2d currPose = mSwerveSubsystem.getPose();
            mXPIDController.reset(currPose.getX());
            mYPIDController.reset(currPose.getY());
            mRotationPIDController.reset(currPose.getRotation().getRadians());
        });
    }

    // Uses a basic PID for fine adjustments to absolute, field-oriented pose
    private void getCloseTo(Pose2d destination, Supplier<VisionDirection> direction)
    {
        // Gets current pose
        Pose2d curPose = mSwerveSubsystem.getPose();

        // Applies transformation based on direction received
        Pose2d fixedPose = destination.transformBy(getTransform(direction.get()));

        ChassisSpeeds speed = new ChassisSpeeds(
            // Calculates X, Y, and rotation error from current pose to destination
            mXPIDController.calculate(curPose.getX(), fixedPose.getX()),
            mYPIDController.calculate(curPose.getY(), fixedPose.getY()),
            mRotationPIDController.calculate(curPose.getRotation().getRadians(), fixedPose.getRotation().getRadians())
        );

        // Drives the robot using calculated speeds
        mSwerveSubsystem.driveFieldOriented(speed);
    }

    // Gets the correct transform for fine PID alignment
    private Transform2d getTransform(VisionDirection endDirection)
    {
        switch (endDirection)
        {
            case LeftCoral:
                return kLeftCoralTransform;
            case RightCoral:
                return kRightCoralTransform;
            case MatchLoading:
                return kMatchLoadingApproachTransform;
            default:
                return kAlgaeTransform;
        }
    }
}
