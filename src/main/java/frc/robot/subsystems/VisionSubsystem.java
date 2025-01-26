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
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.objectmodels.CameraInfo;

public class VisionSubsystem extends SubsystemBase
{
    // TODO: More information!
    public static final CameraInfo[] kInstalledCameras =
    {
        new CameraInfo("Arducam_OV9281_USB_Camera", new Transform3d(new Translation3d(0.3, -0.2, 0.1), new Rotation3d(0, 15, 0))) // Can be changed.
    };

    // Cameras go here.
    // TODO: In simulation, use PhotonCameraSim instead.
    private PhotonCamera[] cameras;

    private Field2d field;
    private AprilTagFieldLayout tagLayout;

    private PhotonPoseEstimator[] poseEstimators;

    // Checks that must pass for vision to be active.
    private boolean initPass;
    private boolean enabled = true;

    private ArrayList<Consumer<EstimatedRobotPose>> poseListeners;

    private EstimatedRobotPose poseApproximation;
    private Optional<PhotonTrackedTarget> bestTarget;
    
    public VisionSubsystem()
    {
        // If anything goes wrong during setup, do not let vision run.
        initPass = true;
        poseListeners = new ArrayList<>();

        try
        {
            // There's a file created by wpilib that has the layout for the new game.
            String layoutPath = AprilTagFields.k2025Reefscape.m_resourceFile;
            System.out.println("[VISION] April Tag Layout Path: " + layoutPath);
            tagLayout = AprilTagFieldLayout.loadFromResource(layoutPath);
        }
        catch (IOException ex)
        {
            // Failed to read layout information.
            // TODO: This fails in simulation, I think because the path is not
            //       valid. Is there a way to avoid this?
            System.out.println("[VISION] FAILED TO LOAD APRIL TAG LAYOUTS! The vision subsystem will NOT be active.");
            initPass = false;
            return;
        }

        field = new Field2d();
        SmartDashboard.putData("Vision Pose", field);

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
        }
    }

    public void addMeasurementListener(Consumer<EstimatedRobotPose> consumer)
    {
        poseListeners.add(consumer);
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putBoolean("Vision Active", isActive());
        if (!isActive()) return; // Disabled.

        bestTarget = Optional.empty();

        // Process vision updates for each camera.
        for (int i = 0; i < cameras.length; i++)
        {
            PhotonCamera camera = cameras[i];
            PhotonPoseEstimator poseEstimator = poseEstimators[i];
            CameraInfo cameraInfo = kInstalledCameras[i];

            // This includes only NEW results.
            // TODO: Maybe previous results are better than the crappy ones we
            //       have now? (Talking on a millisecond-to-millisecond basis)
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
            
            SmartDashboard.putString("Vision: Distance to April Tag", targetPose.minus(curPose).toString());
        }
        else
        {
            SmartDashboard.putString("Vision: Distance to April Tag", "No Tag in View");
        }
    }

    /** Returns the best april tag target in view. */
    public Optional<PhotonTrackedTarget> getTargetInView()
    {
        return bestTarget;
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

    public boolean isActive()
    {
        return initPass && enabled;
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
