package frc.robot.objectmodels;

import edu.wpi.first.math.geometry.Pose2d;

public class VisionPoseInfo
{
    private final Pose2d pose;
    private final double timestamp;

    public VisionPoseInfo(Pose2d pose, double timestamp)
    {
        this.pose = pose;
        this.timestamp = timestamp;
    }

    public Pose2d getPose()
    {
        return pose;
    }
    public double getTimestamp()
    {
        return timestamp;
    }
}
