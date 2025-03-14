package frc.robot.objectmodels;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.VisionSubsystem;

public class VisionPoses {

    public static Pose2d getTargetPose(ReefEndTarget target, VisionSubsystem vision)
    {
        switch (target)
        {
            case NearLeft:
                return getPose(19, 6, vision);
            case NearMiddle:
                return getPose(18, 7, vision);
            case NearRight:
                return getPose(17, 8, vision);
            case FarLeft:
                return getPose(20, 11, vision);
            case FarMiddle:
                return getPose(21, 10, vision);
            case FarRight:
                return getPose(22, 9, vision);
            default: return new Pose2d();
        }
    }

    public static ArrayList<Pose2d> getReefPoses(Transform2d radius, VisionSubsystem vision)
    {
        ArrayList<Pose2d> poses = new ArrayList<Pose2d>();
        int index = isBlueAlliance() ? 17 : 6;

        for (int i = 0; i < 6; i++)
        {
            Pose2d visionPose = vision.getTagLayout().getTagPose(i+index).get().toPose2d();
            Pose2d fixedPose = visionPose.plus(radius);

            poses.add(fixedPose);
        }

        return poses;
    }

    public static Pose2d getLeftMatchLoadingStationPose(VisionSubsystem vision)
    {
        return getPose(13, 1, vision);
    }

    public static Pose2d getRightMatchLoadingStationPose(VisionSubsystem vision)
    {
        return getPose(12, 2, vision);
    }

    public static Pose2d getProcessorPose(VisionSubsystem vision)
    {
        return getPose(16, 3, vision);
    }

    private static Pose2d getPose(int blue, int red, VisionSubsystem vision)
    {
        return vision.getTagLayout().getTagPose(isBlueAlliance() ? blue : red).get().toPose2d();
    }

    private static boolean isBlueAlliance()
    {
        // Returns true if the robot is connected to the FMS and on the blue alliance
        return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Blue);
    } // FIXME: replace with actual helper method in main branch
}
