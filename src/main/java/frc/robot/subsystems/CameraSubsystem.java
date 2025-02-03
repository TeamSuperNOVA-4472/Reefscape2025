package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSubsystem extends SubsystemBase {
    private final PhotonCamera mCamera = new PhotonCamera("Arducam_OV9281_USB_Camera");
    private double mX = 0;
    private double mY = 0;
    private double mZRot = 0;
    private double mYaw = 0;
    private boolean mHasTarget = false;


    public Pose2d getBestTargetOffset() {
        return new Pose2d(mX, mY, Rotation2d.fromDegrees(mZRot));
    }

    public Pose2d calculateOffsetFromDest(Pose2d pTargetDest) {
        if(!mHasTarget || !mCamera.isConnected()) {
            return new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        }
    
        Pose2d bestTargetOffset = getBestTargetOffset();
        double bestTargetRot = ((bestTargetOffset.getRotation().getDegrees() % 360) + 360) % 360;
        double targetDestRot = ((pTargetDest.getRotation().getDegrees() % 360) + 360) % 360;
        double rotationDeg = bestTargetRot - targetDestRot;
        double xMeters =  bestTargetOffset.getX() - pTargetDest.getX();
        double yMeters =  bestTargetOffset.getY() - pTargetDest.getY();
        SmartDashboard.putNumber("Best Target Rot", bestTargetRot);
        SmartDashboard.putNumber("Dest Rot", targetDestRot);
        SmartDashboard.putNumber("New X", xMeters);
        SmartDashboard.putNumber("New Y", yMeters);
        SmartDashboard.putNumber("New Rot", rotationDeg);
        System.out.println("Recalculating!");
        return new Pose2d(xMeters, yMeters, Rotation2d.fromDegrees(rotationDeg));
    }


    @Override
    public void periodic() {
        List<PhotonPipelineResult> results = mCamera.getAllUnreadResults();
        int targetId = -1;
        for(PhotonPipelineResult result : results) {
            if(result.hasTargets()) {
                mHasTarget = true;
                PhotonTrackedTarget target = result.getBestTarget();
                targetId = target.getFiducialId();
                Transform3d transform = target.getBestCameraToTarget();
                mX = transform.getX();
                mY = transform.getY();
                mZRot = Math.toDegrees(transform.getRotation().getZ());
                mYaw = target.getYaw();
                break;
            } else {
                mHasTarget = false;
                mX = 0;
                mY = 0;
                mZRot = 0;
                mYaw = 0;
            }
        }
        SmartDashboard.putNumber("Tag ID", targetId);
        SmartDashboard.putNumber("Target X", mX);
        SmartDashboard.putNumber("Target Y", mY);
        SmartDashboard.putNumber("Target Z Rot", mZRot);
        SmartDashboard.putNumber("Target Yaw", mYaw);
        SmartDashboard.putBoolean("Has Target", mHasTarget);
    }
}
