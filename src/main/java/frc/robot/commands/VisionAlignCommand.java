package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class VisionAlignCommand extends SequentialCommandGroup
{
    private PhotonTrackedTarget activeTarget;

    public VisionAlignCommand(
        SwerveSubsystem pSwerve,
        VisionSubsystem pVision,
        Translation2d pOffsetFromTarget
    )
    {
        AprilTagFieldLayout layout = pVision.getTagLayout();

        // Here's the deal: we're going to run a few commands:
        // - Wait for the vision subsystem to recognize a tag.
        // - Rotate the robot until it aligns.
        // - Wait for the vision to recognize the tag again.
        // - Move the robot to where it needs to be.
        addCommands(
            new WaitForTagCommand(pVision, 2, this::newActiveTarget),
            new InstantCommand(() ->
            {
                if (activeTarget == null)
                {
                    System.out.println("LOOKS LIKE TIME RAN OUT");
                }
                else
                {
                    System.out.print("HERES WHAT I SEE: " + activeTarget.fiducialId);
                }
            })
        );
    }

    private void newActiveTarget(PhotonTrackedTarget seen)
    {
        activeTarget = seen;
    }
}
