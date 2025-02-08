package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class VisionAlignCommand extends SequentialCommandGroup
{
    private PhotonTrackedTarget activeTarget, oldTarget;
    private Pose2d offset;

    public VisionAlignCommand(
        SwerveSubsystem pSwerve,
        VisionSubsystem pVision,
        Translation2d pOffsetFromTarget)
    {
        // TODO: This is ugly. There's a few things that would be nice to clean it up a litte:
        //       - For one, the offset is copy-pasted a few times. Make it a final member variable up here.
        //       - Remove the dumb print statements.
        //       - Bundle each "translation" run into a single method. Do the same for rotation. I've added some comments.
        //       - Come up with a better offset (either for the camera or for here) that actually centers the april tag.
        //       Uhh I think that's it. Maybe make it a little more precise by messing with the PIDs for `DriveDistanceAndHeading`?

        // Here's the deal: we're going to run a few commands:
        // - Wait for the vision subsystem to recognize a tag.
        // - Rotate the robot until it aligns.
        // - Wait for the vision to recognize the tag again.
        // - Move the robot to where it needs to be.
        addCommands(
            new InstantCommand(() ->
            {
                // Initialize.
                activeTarget = null;
                System.out.println("STARTING");
            }),
            new WaitForTagCommand(pVision, () -> -1, 2.0, (seen) -> activeTarget = seen),
            new InstantCommand(() ->
            {
                if (activeTarget == null)
                {
                    // We couldn't spot a tag in a reasonable time,
                    // quit now and don't move.
                    System.out.println("LOOKS LIKE TIME RAN OUT");
                    cancel();
                    return;
                }
                else
                {
                    // Configure the robot to change its rotation.
                    Rotation2d deltaRot = activeTarget.getBestCameraToTarget().getRotation().toRotation2d();
                    System.out.println("I SEE SOMETHING!!! " + activeTarget.fiducialId);
                    System.out.println("Raw rotation: " + deltaRot.getDegrees());

                    double deg = deltaRot.getDegrees();
                    deg = (deg + 540) % 360;

                    System.out.println("Normalized: " + deg);
                    offset = new Pose2d(Translation2d.kZero, Rotation2d.fromDegrees(deg));
                }
            }),
            new DriveDistanceAndHeading(pSwerve, () -> offset),
            new InstantCommand(() ->
            {
                // Reset the tag so we know if we hit it again.
                oldTarget = activeTarget;
                activeTarget = null;
            }),
            new WaitForTagCommand(pVision, () -> oldTarget.fiducialId, 2.0, (seen) -> activeTarget = seen),
            new InstantCommand(() ->
            {
                System.out.println("ROUND 2 DING DING DING");
                if (activeTarget == null)
                {
                    // We couldn't spot a tag in a reasonable time,
                    // quit now and don't move.
                    System.out.println("LOOKS LIKE TIME RAN OUT");
                    cancel();
                    return;
                }
                else
                {
                    // TODO: Put this somewhere else.
                    final Translation2d tagOffset = new Translation2d(0.4, 0);

                    // Configure the robot to change its rotation.
                    Translation2d deltaPos = activeTarget.getBestCameraToTarget().getTranslation().toTranslation2d();
                    System.out.println("I SEE SOMETHING!!! " + activeTarget.fiducialId);
                    System.out.println("Raw rotation: " + deltaPos.toString());

                    deltaPos = deltaPos.minus(tagOffset);

                    System.out.println("With offset: " + deltaPos);
                    offset = new Pose2d(deltaPos, Rotation2d.kZero);
                }
            }),
            new DriveDistanceAndHeading(pSwerve, () -> offset),
            new InstantCommand(() ->
            {
                // Reset the tag so we know if we hit it again.
                oldTarget = activeTarget;
                activeTarget = null;
            }),
            new WaitForTagCommand(pVision, () -> -1, 2.0, (seen) -> activeTarget = seen),
            new InstantCommand(() ->
            {
                if (activeTarget == null)
                {
                    // We couldn't spot a tag in a reasonable time,
                    // quit now and don't move.
                    System.out.println("LOOKS LIKE TIME RAN OUT");
                    cancel();
                    return;
                }
                else
                {
                    // Configure the robot to change its rotation.
                    Rotation2d deltaRot = activeTarget.getBestCameraToTarget().getRotation().toRotation2d();
                    System.out.println("I SEE SOMETHING!!! " + activeTarget.fiducialId);
                    System.out.println("Raw rotation: " + deltaRot.getDegrees());

                    double deg = deltaRot.getDegrees();
                    deg = (deg + 540) % 360;

                    System.out.println("Normalized: " + deg);
                    offset = new Pose2d(Translation2d.kZero, Rotation2d.fromDegrees(deg));
                }
            }),
            new DriveDistanceAndHeading(pSwerve, () -> offset),
            new InstantCommand(() ->
            {
                // Reset the tag so we know if we hit it again.
                oldTarget = activeTarget;
                activeTarget = null;
            }),
            new WaitForTagCommand(pVision, () -> oldTarget.fiducialId, 2.0, (seen) -> activeTarget = seen),
            new InstantCommand(() ->
            {
                System.out.println("ROUND 3 DING DING DING");
                if (activeTarget == null)
                {
                    // We couldn't spot a tag in a reasonable time,
                    // quit now and don't move.
                    System.out.println("LOOKS LIKE TIME RAN OUT");
                    cancel();
                    return;
                }
                else
                {
                    // TODO: Put this somewhere else.
                    final Translation2d tagOffset = new Translation2d(0.4, 0);

                    // Configure the robot to change its rotation.
                    Translation2d deltaPos = activeTarget.getBestCameraToTarget().getTranslation().toTranslation2d();
                    System.out.println("I SEE SOMETHING!!! " + activeTarget.fiducialId);
                    System.out.println("Raw rotation: " + deltaPos.toString());

                    deltaPos = deltaPos.minus(tagOffset);

                    System.out.println("With offset: " + deltaPos);
                    offset = new Pose2d(deltaPos, Rotation2d.kZero);
                }
            }),
            new DriveDistanceAndHeading(pSwerve, () -> offset),
            new InstantCommand(() ->
            {
                // Reset the tag so we know if we hit it again.
                oldTarget = activeTarget;
                activeTarget = null;
            }),
            new WaitForTagCommand(pVision, () -> oldTarget.fiducialId, 2.0, (seen) -> activeTarget = seen),
            new InstantCommand(() ->
            {
                System.out.println("ROUND 4 DING DING DING");
                if (activeTarget == null)
                {
                    // We couldn't spot a tag in a reasonable time,
                    // quit now and don't move.
                    System.out.println("LOOKS LIKE TIME RAN OUT");
                    cancel();
                    return;
                }
                else
                {
                    // TODO: Put this somewhere else.
                    final Translation2d tagOffset = new Translation2d(0.4, 0);

                    // Configure the robot to change its rotation.
                    Translation2d deltaPos = activeTarget.getBestCameraToTarget().getTranslation().toTranslation2d();
                    System.out.println("I SEE SOMETHING!!! " + activeTarget.fiducialId);
                    System.out.println("Raw rotation: " + deltaPos.toString());

                    deltaPos = deltaPos.minus(tagOffset);

                    System.out.println("With offset: " + deltaPos);
                    offset = new Pose2d(deltaPos, Rotation2d.kZero);
                }
            }),
            new DriveDistanceAndHeading(pSwerve, () -> offset)
        );
    }
}
