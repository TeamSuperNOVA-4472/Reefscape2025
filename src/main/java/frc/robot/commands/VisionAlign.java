package frc.robot.commands;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

import org.opencv.video.TrackerDaSiamRPN;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
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
import frc.robot.objectmodels.CarriagePreset;
import frc.robot.objectmodels.ReefEndTarget;
import frc.robot.objectmodels.VisionDirection;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// TODO:
// - fix blue alliance bugs
// - debug PID stop/start issue
// - use instance variables and make methods static
// I'll get these done in the car tomorrow.

public class VisionAlign {
    // Member variables
    private final SwerveSubsystem mSwerveSubsystem;
    private final VisionSubsystem mVisionSubsystem;

    private final ProfiledPIDController mXPIDController;
    private final ProfiledPIDController mYPIDController;
    private final PIDController mRotationPIDController;
    
    // Constants
    private final Transform2d kWayPointTransform = new Transform2d(1.5, 0, Rotation2d.k180deg); // Controls radius of reef-avoiding
    private final Transform2d kBackUpTransform = new Transform2d(-0.5, 0, new Rotation2d()); // Back up from stations

    private final Transform2d kReefTransform = new Transform2d(0.75, 0, Rotation2d.k180deg); // Position at reef
    private final Transform2d kMatchLoadingTransform = new Transform2d(1.5, 0, Rotation2d.k180deg); // Position at match loading station
    private final Transform2d kProcessorTransform = new Transform2d(1, 0, Rotation2d.k180deg); // Position at processor

    // Left coral transforms go here
    // If it's easier for you guys and the y values are the same, 
    // I can consolidate Left/Right into one big map that negates the Y value.
    // But that's up to y'all.
    private final Transform2d kLeftCoralL1Transform = new Transform2d(0.5, 0, Rotation2d.k180deg);
    private final Transform2d kLeftCoralL2Transform = new Transform2d(0.5, -0.265, Rotation2d.k180deg);
    private final Transform2d kLeftCoralL3Transform = new Transform2d(0.5, -0.25, Rotation2d.k180deg); 
    private final Transform2d kLeftCoralL4Transform = new Transform2d(0.45, -0.3, Rotation2d.k180deg); 

    // Right coral transforms
    private final Transform2d kRightCoralL1Transform = new Transform2d(0.5, 0, Rotation2d.k180deg); 
    private final Transform2d kRightCoralL2Transform = new Transform2d(0.5, 0.21, Rotation2d.k180deg);
    private final Transform2d kRightCoralL3Transform = new Transform2d(0.5, 0.25, Rotation2d.k180deg);
    private final Transform2d kRightCoralL4Transform = new Transform2d(0.45, 0.3, Rotation2d.k180deg);

    // Algae transforms
    private final Transform2d kAlgaeTopTransform = new Transform2d(0.75, 0, Rotation2d.k180deg); // Position at L3
    private final Transform2d kAlgaeBottomTransform = new Transform2d(0.75, 0, Rotation2d.k180deg); // Position at L2

    private final Transform2d kMatchLoadingApproachTransform = new Transform2d(0.75, 0, new Rotation2d());

    // Maps for CarriagePreset transforms
    private final HashMap<CarriagePreset, Transform2d> kLeftCoralTransforms = new HashMap<>();
    private final HashMap<CarriagePreset, Transform2d> kRightCoralTransforms = new HashMap<>();
    private final HashMap<CarriagePreset, Transform2d> kAlgaeTransforms = new HashMap<>();

    private final double kMaxVelocity = 2.5; // Max velocity for PathPlanner
    private final double kMaxAcceleration = 2.5; // Max acceleration for PathPlanner

    private final double kEndVelocity = 0; // Ending velocity for PathPlanner

    // PID Constants for the Lateral PID Controller
    private final double kLateralP = 1.5;
    private final double kLateralI = 0;
    private final double kLateralD = 0.05;
    private final double kLateralMaxVelocity = 4.5;
    private final double kLateralMaxAcceleration = 4.5;
    private final double kLateralTolerance = 0.02;

    // PID Constants for the Rotation PID Controller
    private final double kRotationP = 0.75;
    private final double kRotationI = 0;
    private final double kRotationD = 0;
    private final double kRotationMaxVelocity = 2;
    private final double kRotationMaxAcceleration = 2;
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

        mXPIDController = new ProfiledPIDController(kLateralP, kLateralI, kLateralD, new Constraints(kLateralMaxVelocity, kLateralMaxAcceleration));
        mYPIDController = new ProfiledPIDController(kLateralP, kLateralI, kLateralD, new Constraints(kLateralMaxVelocity, kLateralMaxAcceleration));
        mRotationPIDController = new PIDController(kRotationP, kRotationI, kRotationD);

        mRotationPIDController.enableContinuousInput(kMinContinuous, kMaxContinuous);

        // Assign PID tolerances
        mXPIDController.setTolerance(kLateralTolerance);
        mYPIDController.setTolerance(kLateralTolerance);
        mRotationPIDController.setTolerance(kRotationTolerance);

        // Assign left coral transforms
        kLeftCoralTransforms.put(CarriagePreset.kCoralL1, kLeftCoralL1Transform);
        kLeftCoralTransforms.put(CarriagePreset.kCoralL2, kLeftCoralL2Transform);
        kLeftCoralTransforms.put(CarriagePreset.kCoralL3, kLeftCoralL3Transform);
        kLeftCoralTransforms.put(CarriagePreset.kCoralL4, kLeftCoralL4Transform);

        // Assign right coral transforms
        kRightCoralTransforms.put(CarriagePreset.kCoralL1, kRightCoralL1Transform);
        kRightCoralTransforms.put(CarriagePreset.kCoralL2, kRightCoralL2Transform);
        kRightCoralTransforms.put(CarriagePreset.kCoralL3, kRightCoralL3Transform);
        kRightCoralTransforms.put(CarriagePreset.kCoralL4, kRightCoralL4Transform);

        // Assign algae transforms
        kAlgaeTransforms.put(CarriagePreset.kAlgaeL2, kAlgaeBottomTransform);
        kAlgaeTransforms.put(CarriagePreset.kAlgaeL3, kAlgaeTopTransform);
    }

    // Aligns to the nearest reef face
    private Command alignToNearestReef(Supplier<VisionDirection> direction, Supplier<Optional<CarriagePreset>> preset)
    {
        ArrayList<Pose2d> poses = VisionPoses.getReefPoses(new Transform2d(), mVisionSubsystem); // Creates list of poses
        Pose2d currPose = mSwerveSubsystem.getPose(); // Gets current pose
        Pose2d destination = currPose.nearest(poses); // Gets nearest reef pose to current pose

        return getCloseToCommand(destination, () -> getReefTransform(direction, preset));
    }

    /**
     * Aligns to the nearest reef face from field view for teleop.
     * @param leftButton
     * @param rightButton
     * @param preset
     * @return
     */
    public Command alignToNearestReef(Supplier<Boolean> leftButton, Supplier<Boolean> rightButton, Supplier<Optional<CarriagePreset>> preset)
    {
        Supplier<VisionDirection> direction = () -> getOffset(leftButton, rightButton);

        return alignToNearestReef(direction, preset);
    }

    public Command alignToNearestReef(VisionDirection direction, Supplier<Optional<CarriagePreset>> preset)
    {
        return alignToNearestReef(() -> direction, preset);
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
            Pose2d idealWaypoint = destination.plus(kWayPointTransform);
            Command pathFind = pathFindToPlace(idealWaypoint, exitPoint, kWayPointTransform);
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
     * @param preset The supplier that gives the desired preset of the robot.
     * @return The sequence of pathfinding and PID alignment.
     */
    public SequentialCommandGroup alignToReef(ReefEndTarget target, Supplier<Boolean> leftButton, Supplier<Boolean> rightButton, Supplier<Optional<CarriagePreset>> preset)
    {
        Pose2d destination = VisionPoses.getTargetPose(target, mVisionSubsystem);
        Supplier<VisionDirection> direction = () -> getOffset(leftButton, rightButton);

        Command getClose = getCloseToCommand(destination, () -> getReefTransform(direction, preset));

        return alignToReef(target, getClose);
    }

    /**
     * Pathfinds and aligns to a face on the reef.
     * @param target The enum describing end target from field-oriented position.
     * @param direction The enum describing left, right, or middle PID alignment.
     * @param preset The supplier that gives the desired preset of the robot.
     * @return The sequence of pathfinding and PID alignment.
     */
    public SequentialCommandGroup alignToReef(ReefEndTarget target, VisionDirection direction, Supplier<Optional<CarriagePreset>> preset)
    {
        Pose2d destination = VisionPoses.getTargetPose(target, mVisionSubsystem);
        Command getClose = getCloseToCommand(destination, getReefTransform(() -> direction, preset));
        
        return alignToReef(target, getClose);
    }

    // Consolidates both align to match loading station commnads
    private SequentialCommandGroup alignToMatchLoadingStation(Pose2d destination)
    {
        destination = destination.plus(kMatchLoadingTransform); // Adds desired transform
        Pose2d target = destination.nearest(VisionPoses.getReefPoses(kWayPointTransform, mVisionSubsystem)); // Finds nearest pose around reef

        Command approach = getCloseToCommand(destination, () -> kMatchLoadingApproachTransform);
        
        try {
            return new SequentialCommandGroup(pathFindToPlace(target, destination, kWayPointTransform), approach); // If enough waypoints are present, runs
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

        Pose2d currPose = mSwerveSubsystem.getPose().transformBy(kBackUpTransform); // Gets the current position of the drive
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
            new GoalEndState(kEndVelocity, lastPose.getRotation())
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
    private Command getCloseToCommand(Pose2d destination, Supplier<Transform2d> direction)
    {
        Command runCommand  =
            new RunCommand(() -> getCloseTo(destination, direction), mSwerveSubsystem, mVisionSubsystem);

        // Create command that resets PID before running everything else
        return new SequentialCommandGroup(
            resetProfiledPidCommand(),
            runCommand
        );
    }

    // Creates the PID command that runs until controllers are at setpoint
    private Command getCloseToCommand(Pose2d destination, Transform2d direction)
    {
        Command command = new RunCommand(() -> getCloseTo(destination, () -> direction), mSwerveSubsystem, mVisionSubsystem)
            .until(() ->
                // Exits once PID controllers are at goal
                mXPIDController.atGoal() &&
                mYPIDController.atGoal() &&
                mRotationPIDController.atSetpoint()
            );

            // Create command that resets PID before running everything else
            return new SequentialCommandGroup(
                resetProfiledPidCommand(),
                command);
    }

    // TODO: debug weird jank movement between pathfinding and PID. It's worth putting the PID reset into
    // a parallel group with pathfinding and having it run repeatedly.
    // Resets the profiled PID to its correct starting position
    private Command resetProfiledPidCommand() {
        // Create new command
        return new InstantCommand(() -> {
            // Get current pose and velocity
            Pose2d currPose = mSwerveSubsystem.getPose();
            ChassisSpeeds currSpeed = mSwerveSubsystem.getRobotRelativeSpeeds();

            // Reset controllers with position and velocity
            mXPIDController.reset(currPose.getX(), currSpeed.vxMetersPerSecond);
            mYPIDController.reset(currPose.getY(), currSpeed.vyMetersPerSecond);
        });
    }

    // Uses a basic PID for fine adjustments to absolute, field-oriented pose
    private void getCloseTo(Pose2d destination, Supplier<Transform2d> direction)
    {
        // Gets current pose
        Pose2d curPose = mSwerveSubsystem.getPose();

        // Applies transformation based on direction received
        Pose2d fixedPose = destination.transformBy(direction.get());

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
    private Transform2d getReefTransform(Supplier<VisionDirection> endDirection, Supplier<Optional<CarriagePreset>> presetGetter)
    {
        // Get the desired preset
        Optional<CarriagePreset> desiredPreset = presetGetter.get();

        // If no preset is present, assume it's in the away position
        CarriagePreset preset = desiredPreset.isPresent() ? desiredPreset.get() : CarriagePreset.kAway;

        switch (endDirection.get())
        {
            case LeftCoral:
                // Get the correct transform if a reef preset is present; if not, assume L1
                return kLeftCoralTransforms.getOrDefault(preset, kLeftCoralL1Transform);
            case RightCoral:
                // Get the correct transform if a reef preset is present; if not, assume L1
                return kRightCoralTransforms.getOrDefault(preset, kRightCoralL1Transform);
            default:
                // Return algae otherwise
                // Check if algae is present; if not, assume L2 algae
                return kAlgaeTransforms.getOrDefault(preset, kAlgaeBottomTransform);
        }
    }
}
