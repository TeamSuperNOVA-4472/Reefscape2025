package frc.robot.commands;

import java.util.Optional;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/** A command that aligns the robot to an april tag in front of itself.
    Sophia came up with a bunch of the logic behind this, credit to her.
    The command does several iterations of locating an april tag and matching
    either its rotation or its translation. The several iterations improve
    accuracy but increase duration. Worth tweaking. */

// FIXME: While the `WaitForTagCommand` can focus on a specific tag,
//        it only pays attention to the best one visible. So, for example,
//        if there are two tags on the camera, even if the correct one is
//        visible, it may focus on the other one and "fail to find the tag."

// FIXME: This causes a crash when a tag is not spotted during an autonomous
//        path. Hopefully our tuning will make this never happen, but it could
//        be problematic. Somehow, the `cancel()` method isn't doing its job
//        when used with PathPlanner.
public class VisionAlignCommand extends SequentialCommandGroup
{
    private PhotonTrackedTarget activeTarget, oldTarget;
    private Pose2d drivePerIterOffset;

    // The final offset from the center of the robot to the target.
    private Translation2d offsetFromTarget;

    // Represents the camera offset to the robot center. TWEAK
    private final Translation2d camOffsetToRobotCenter = new Translation2d(0.6, -0.30); // 0.267

    // Used for print statements.
    private int translationIter = 0, rotationIter = 0;

    // Extra stuff.
    private Optional<Runnable> runOnComplete;

    public VisionAlignCommand(
        SwerveSubsystem pSwerve,
        VisionSubsystem pVision,
        Translation2d pOffsetFromTarget,
        Optional<Runnable> pRunOnComplete)
    {
        offsetFromTarget = pOffsetFromTarget;
        runOnComplete = pRunOnComplete;

        // Here's the deal: we're going to run a few commands:
        // - Wait for the vision subsystem to recognize a tag.
        // - Rotate the robot until it aligns.
        // - Wait for the vision to recognize the tag again.
        // - Move the robot to where it needs to be.
        // Repeat a few times until we're within a reasonable distance.

        // This could be simplified even further to a bunch of methods that automatically add the chunks
        // as a whole. Prevents duplicate code, but I think this is good enough for now.
        addCommands(
            new InstantCommand(this::alignInitialize),

            // Rotation step 1
            new InstantCommand(this::alignReset), // This line not technically required, but it serves as a reference.
            new WaitForTagCommand(pVision, () -> -1, 2.0, (seen) -> activeTarget = seen),
            new InstantCommand(this::alignRotation),
            new DriveDistanceAndHeading(pSwerve, () -> drivePerIterOffset),

            // Translation step 1
            new InstantCommand(this::alignReset),
            new WaitForTagCommand(pVision, () -> oldTarget.fiducialId, 2.0, (seen) -> activeTarget = seen),
            new InstantCommand(this::alignTranslation),
            new DriveDistanceAndHeading(pSwerve, () -> drivePerIterOffset),

            // Rotation step 2
            new InstantCommand(this::alignReset),
            new WaitForTagCommand(pVision, () -> -1, 2.0, (seen) -> activeTarget = seen),
            new InstantCommand(this::alignRotation),
            new DriveDistanceAndHeading(pSwerve, () -> drivePerIterOffset),

            // Translation step 2
            new InstantCommand(this::alignReset),
            new WaitForTagCommand(pVision, () -> oldTarget.fiducialId, 2.0, (seen) -> activeTarget = seen),
            new InstantCommand(this::alignTranslation),
            new DriveDistanceAndHeading(pSwerve, () -> drivePerIterOffset),

            // Translation step 3
            new InstantCommand(this::alignReset),
            new WaitForTagCommand(pVision, () -> oldTarget.fiducialId, 2.0, (seen) -> activeTarget = seen),
            new InstantCommand(this::alignTranslation),
            new DriveDistanceAndHeading(pSwerve, () -> drivePerIterOffset),

            // One more thing. Apply the offset.
            new DriveDistanceAndHeading(pSwerve, () -> new Pose2d(offsetFromTarget, Rotation2d.kZero)),

            // Runnable to invoke on completion of the command.
            // Sophia's code again, just condensed a little.
            new InstantCommand(() -> { if (runOnComplete.isPresent()) runOnComplete.get().run(); })
        );
    }

    private void alignInitialize()
    {
        // Initialize.
        activeTarget = null;
        translationIter = 0;
        rotationIter = 0;

        System.out.println("[ALIGN] Vision align to reef begun.");
    }
    private void alignReset()
    {
        // Reset the tag so we know if we hit it again.
        oldTarget = activeTarget;
        activeTarget = null;
    }

    private void alignTranslation()
    {
        translationIter++;
        if (activeTarget == null)
        {
            // We couldn't spot a tag in a reasonable time,
            // quit now and don't move.
            System.out.printf("[ALIGN] Translation step %d has failed!\n", translationIter);
            cancel();
            return;
        }
        else
        {
            // TODO: Put this somewhere else.
            // Configure the robot to change its rotation.
            Translation2d deltaPos = activeTarget.getBestCameraToTarget().getTranslation().toTranslation2d();
            System.out.printf("[ALIGN] Translation step %d spots AprilTag #%d\n", translationIter, activeTarget.fiducialId);
            
            deltaPos = deltaPos.minus(camOffsetToRobotCenter);
            drivePerIterOffset = new Pose2d(deltaPos, Rotation2d.kZero);
        }
    }
    private void alignRotation()
    {
        rotationIter++;
        if (activeTarget == null)
        {
            // We couldn't spot a tag in a reasonable time,
            // quit now and don't move.
            System.out.printf("[ALIGN] Rotation step %d has failed!\n", rotationIter);
            cancel();
            return;
        }
        else
        {
            // Configure the robot to change its rotation.
            Rotation2d deltaRot = activeTarget.getBestCameraToTarget().getRotation().toRotation2d();
            System.out.printf("[ALIGN] Rotation step %d spots AprilTag #%d\n", rotationIter, activeTarget.fiducialId);

            // Sophia's idea. Add 180 extra degrees.
            // I want to switch this back to Rotation2d math, but
            // better to leave it like this for now so we don't break stuff.
            double deg = deltaRot.getDegrees();
            deg = (deg + 540) % 360;

            drivePerIterOffset = new Pose2d(Translation2d.kZero, Rotation2d.fromDegrees(deg));
        }
    }
}
