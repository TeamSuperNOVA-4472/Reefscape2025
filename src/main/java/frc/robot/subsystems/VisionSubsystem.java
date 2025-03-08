package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.objectmodels.CameraInfo;

public class VisionSubsystem extends SubsystemBase
{
    // TODO: CHANGE BACK TO OG
    // The first camera in this array is considered the "main camera."
    // TODO: More information for more cameras!
    public static final CameraInfo[] kInstalledCameras =
    {
        new CameraInfo("Arducam_OV9281_USB_Camera", new Transform3d(new Translation3d(0.158671, -0.206121, 0.26035 ), new Rotation3d(0, -0.393, 0))) // Can be changed.
    };

    // Cameras go here.
    private PhotonCamera[] cameras;

    private Field2d field;
    private AprilTagFieldLayout tagLayout;

    private PhotonPoseEstimator[] poseEstimators;

    // Checks that must pass for vision to be active.
    private boolean initPass;
    private boolean mainConnectedPass;
    private boolean enabled = true;

    private ArrayList<Consumer<EstimatedRobotPose>> poseListeners;

    private EstimatedRobotPose poseApproximation;
    private Optional<PhotonTrackedTarget> bestTarget;
    private PhotonTrackedTarget lastSeenTarget;

    // Simulation
    private VisionSystemSim simVision;
    private PhotonCameraSim[] simCameras;
    // FIXME: DEBUG DELETE ASAP
    private SwerveSubsystem mSwerve;

    private Timer statusCheckTimer;
    
    public VisionSubsystem(SwerveSubsystem pSwerve)
    {
        // If anything goes wrong during setup, do not let vision run.
        initPass = true;
        poseListeners = new ArrayList<>();

        try
        {
            // There's a file created by wpilib that has the layout for the new game.
            String layoutPath = AprilTagFields.k2025ReefscapeAndyMark.m_resourceFile;
            System.out.println("[VISION] April Tag Layout Path: " + layoutPath);
            tagLayout = AprilTagFieldLayout.loadFromResource(layoutPath);
        }
        catch (IOException ex)
        {
            // Failed to read layout information.
            System.out.println("[VISION] FAILED TO LOAD APRIL TAG LAYOUTS! The vision subsystem will NOT be active.");
            initPass = false;
            return;
        }

        field = new Field2d();
        SmartDashboard.putData("Subsystems/VisionSubsystem/Vision Pose", field);
        mSwerve = pSwerve;

        // Initialize cameras. This means we don't need a million variables for
        // all the cameras.
        cameras = new PhotonCamera[kInstalledCameras.length];
        poseEstimators = new PhotonPoseEstimator[kInstalledCameras.length];
        for (int i = 0; i < cameras.length; i++)
        {
            CameraInfo info = kInstalledCameras[i];
            cameras[i] = new PhotonCamera(info.getName());
            poseEstimators[i] = new PhotonPoseEstimator(
                tagLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                info.getOffset());
            poseEstimators[i].setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        }

        if (RobotBase.isSimulation())
        {
            simVision = new VisionSystemSim("Test");
            simVision.addAprilTags(tagLayout);
            simCameras = new PhotonCameraSim[kInstalledCameras.length];
            for (int i = 0; i < cameras.length; i++)
            {
                simCameras[i] = new PhotonCameraSim(cameras[i]);
                simVision.addCamera(simCameras[i], kInstalledCameras[i].getOffset());
                simCameras[i].enableRawStream(true);
                simCameras[i].enableProcessedStream(true);
                simCameras[i].enableDrawWireframe(true);
            }
        }

        statusCheckTimer = new Timer();
        statusCheckTimer.start();
        checkCameraStatus();
    }

    public void addMeasurementListener(Consumer<EstimatedRobotPose> consumer)
    {
        poseListeners.add(consumer);
    }

    private void checkCameraStatus()
    {
        boolean mainBad = false;
        ArrayList<Integer> badCameras = new ArrayList<>();
        for (int i = 0; i < cameras.length; i++)
        {
            PhotonCamera cam = cameras[i];
            if (!cam.isConnected())
            {
                if (i == 0) mainBad = true;
                else badCameras.add(i);
            }
        }

        mainConnectedPass = !mainBad;
        if (mainBad)
        {
            System.out.println("[VISION] The main camera is not connected! Vision will NOT be active.");
        }

        if (badCameras.size() > 0)
        {
            // Print message for cameras.
            StringBuilder msg = new StringBuilder("[VISION] The following tertiary cameras are not connected: ");
            for (int i = 0; i < badCameras.size(); i++)
            {
                msg.append('#');
                msg.append(badCameras.get(i));
                if (i < badCameras.size() - 1) msg.append(", ");
            }
        }
    }

    @Override
    public void periodic()
    {
        if (statusCheckTimer.hasElapsed(1))
        {
            checkCameraStatus();
            statusCheckTimer.restart();
        }

        SmartDashboard.putBoolean("Subsystems/VisionSubsystem/Vision Status", isActive());
        if (!isActive()) return; // Disabled.

        bestTarget = Optional.empty();

        // Process vision updates for each camera.
        for (int i = 0; i < cameras.length; i++)
        {
            PhotonCamera camera = cameras[i];
            PhotonPoseEstimator poseEstimator = poseEstimators[i];

            List<PhotonPipelineResult> results = camera.getAllUnreadResults();
            Optional<EstimatedRobotPose> newRobotPose = Optional.empty();
            for (PhotonPipelineResult result : results)
            {
                if (!result.hasTargets()) continue;
                
                // Get pose offset. If the tag is hard-coded (which it is) and
                // we know our offset to the tag, we can determine our absolute
                // position in the field.
                // TODO: Maybe we should include more than one "good target?"
                PhotonTrackedTarget target = result.getBestTarget();
                int tagId = target.getFiducialId();
                Optional<Pose3d> tagPose = tagLayout.getTagPose(tagId);
                if (tagPose.isEmpty()) continue; // Pose was not found. Does this actually happen?
                
                // Use offset to approximate actual robot position.
                newRobotPose = poseEstimator.update(result);
            
                // We *should* have a decent approximation. We're done here.
                bestTarget = Optional.of(target);
                lastSeenTarget = target;
            }
            if  (newRobotPose.isPresent())
            {
                EstimatedRobotPose pose = newRobotPose.get();
                updatePose(pose);
            }
        }

        // DEBUG: Distance to best april tag.
        Optional<PhotonTrackedTarget> best = getTargetInView();
        if (best.isPresent())
        {
            PhotonTrackedTarget target = best.get();
            Pose3d curPose = poseApproximation.estimatedPose;
            Pose3d targetPose = tagLayout.getTagPose(target.getFiducialId()).get();
            
            SmartDashboard.putString("Subsystems/VisionSubsystem/Vision Distance to April Tag", targetPose.minus(curPose).toString());
        }
        else
        {
            SmartDashboard.putString("Subsystems/VisionSubsystem/Vision Distance to April Tag", "No Tag in View");
        }

        if (getPoseInfo()!=null&&getTargetInView().isPresent())
        {
            Pose3d dest = tagLayout.getTagPose(getTargetInView().get().fiducialId).get();
            double destDeg = ((dest.getRotation().toRotation2d().getDegrees())+360)%360;
            double currentDeg = (getPose().getRotation().toRotation2d().getDegrees()+360)%360;
            SmartDashboard.putNumber("Subsystems/VisionSubsystem/Vision Rotation to April Tag", destDeg);
            SmartDashboard.putNumber("Subsystems/VisionSubsystem/Vision Rotation DT", currentDeg);
            if (getTargetInView().isPresent())
            {
                SmartDashboard.putNumber("Subsystems/VisionSubsystem/Vision mY", getCameraToTarget(getTargetInView().get()).getY());
                SmartDashboard.putNumber("Subsystems/VisionSubsystem/Vision difference in Y", dest.getY() - getPose().getY());
            }
        }

        if (getPoseInfo()!=null)
        {
            SmartDashboard.putString("Subsystems/VisionSubsystem/Vision Pose String: ", getPose().toString());
        }

        if (RobotBase.isSimulation())
        {
            Pose2d currentPose = mSwerve.getPose();
            simVision.update(new Pose3d(currentPose));
        }
    }

    /** Returns the best april tag target in view. */
    public Optional<PhotonTrackedTarget> getTargetInView()
    {
        return bestTarget;
    }

    public Transform3d getCameraToTarget(PhotonTrackedTarget target)
    {
        return target.getBestCameraToTarget();
   
    }

    private void updatePose(EstimatedRobotPose newPose)
    {
        poseApproximation = newPose;
        field.setRobotPose(newPose.estimatedPose.toPose2d());
        for (Consumer<EstimatedRobotPose> callback : poseListeners) callback.accept(newPose);
    }

    public EstimatedRobotPose getPoseInfo()
    {
        return poseApproximation;
    }
    public Pose3d getPose()
    {
        return poseApproximation.estimatedPose;
    }

    public double getPoseTimestamp()
    {
        return poseApproximation.timestampSeconds;
    }
    
    public AprilTagFieldLayout getTagLayout()
    {
        return tagLayout;
    }

    public PhotonTrackedTarget getLastSeenTarget()
    {
        return lastSeenTarget;
    }

    public boolean isActive()
    {
        if (RobotBase.isSimulation())
        {
            return initPass && enabled;
        }
        return initPass && mainConnectedPass && enabled;
    }
    public void enable()
    {
        enabled = true;
    }
    public void disable()
    {
        enabled = false;
    }
}
