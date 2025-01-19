package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.objectmodels.CameraInfo;

public class VisionSubsystem extends SubsystemBase
{
    // TODO: More information!
    public static final CameraInfo[] kInstalledCameras =
    {
        new CameraInfo("Arducam_OV9281_USB_Camera", Transform3d.kZero) // Can be changed.
    };

    // Cameras go here.
    // TODO: In simulation, use PhotonCameraSim instead.
    private PhotonCamera[] cameras;

    private Field2d field;
    private AprilTagFieldLayout tagLayout;

    // Checks that must pass for vision to be active.
    private boolean initPass;
    private boolean enabled = true;

    private ArrayList<Consumer<Pose2d>> poseListeners;

    private Pose2d poseApproximation;
    
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
            tagLayout = new AprilTagFieldLayout(layoutPath);
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
        for (int i = 0; i < cameras.length; i++)
        {
            CameraInfo info = kInstalledCameras[i];
            cameras[i] = new PhotonCamera(info.getName());
        }
    }

    public void addPoseConsumer(Consumer<Pose2d> consumer)
    {
        poseListeners.add(consumer);
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putBoolean("Vision Active", isActive());
        if (!isActive()) return; // Disabled.

        // Process vision updates for each camera.
        for (int i = 0; i < cameras.length; i++)
        {
            PhotonCamera camera = cameras[i];
            CameraInfo cameraInfo = kInstalledCameras[i];

            // This includes only NEW results.
            // TODO: Maybe previous results are better than the crappy ones we
            //       have now? (Talking on a millisecond-to-millisecond basis)
            List<PhotonPipelineResult> results = camera.getAllUnreadResults();
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
                // TODO: Using PhotonPoseEstimator is more accurate but seems more complex. Will do it later.
                Pose3d robotPose3d = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), tagPose.get(), cameraInfo.getOffset());
                Pose2d robotPose2d = new Pose2d(robotPose3d.getTranslation().toTranslation2d(),
                                                robotPose3d.getRotation().toRotation2d());

                // We *should* have a decent approximation. We're done here.
                updatePose(robotPose2d);
            }
        }
    }

    private void updatePose(Pose2d newPose)
    {
        poseApproximation = newPose;
        field.setRobotPose(newPose);
        for (Consumer<Pose2d> callback : poseListeners) callback.accept(newPose);
    }

    public Pose2d getRobotPosition()
    {
        return poseApproximation;
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
