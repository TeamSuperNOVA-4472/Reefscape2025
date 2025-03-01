package frc.robot.commands;

import java.util.Optional;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.objectmodels.LightState;
import frc.robot.objectmodels.LightStatusRequest;
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
public class VisionAlignCommand extends SequentialCommandGroup
{
    public static final Translation2d kReefLeftOffset = new Translation2d(0.16, 0.38);
    public static final Translation2d kReefMiddleOffset = new Translation2d(0.2, 0.215);
    public static final Translation2d kReefRightOffset = new Translation2d(0.22, 0.02);

    public static final Translation2d kDeltaForL1 = new Translation2d(0, 0);
    public static final Translation2d kDeltaForL2 = new Translation2d(0, 0);
    public static final Translation2d kDeltaForL3 = new Translation2d(0.07, 0);
    public static final Translation2d kDeltaForL4 = new Translation2d(0, 0);

    public static final Translation2d kDeltaForAlgae = new Translation2d(0.2, 0);

    private PhotonTrackedTarget activeTarget, oldTarget;
    private Pose2d drivePerIterOffset;

    // The final offset from the center of the robot to the target.
    private Translation2d offsetFromTarget;

    // Represents the camera offset to the robot center. TWEAK
    private final Translation2d camOffsetToRobotCenter = new Translation2d(0.6, -0.15);

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
            new WaitForTagCommand(pVision, this::passDesiredTarget, 2.0, (seen) -> activeTarget = seen),
            new InstantCommand(this::alignRotation),
            new DriveDistanceAndHeading(pSwerve, () -> drivePerIterOffset),

            // Translation step 1
            new InstantCommand(this::alignReset),
            new WaitForTagCommand(pVision, this::passDesiredTarget, 2.0, (seen) -> activeTarget = seen),
            new InstantCommand(this::alignTranslation),
            new DriveDistanceAndHeading(pSwerve, () -> drivePerIterOffset),

            // Rotation step 2
            new InstantCommand(this::alignReset),
            new WaitForTagCommand(pVision, this::passDesiredTarget, 2.0, (seen) -> activeTarget = seen),
            new InstantCommand(this::alignRotation),
            new DriveDistanceAndHeading(pSwerve, () -> drivePerIterOffset),

            // Translation step 2
            new InstantCommand(this::alignReset),
            new WaitForTagCommand(pVision, this::passDesiredTarget, 2.0, (seen) -> activeTarget = seen),
            new InstantCommand(this::alignTranslation),
            new DriveDistanceAndHeading(pSwerve, () -> drivePerIterOffset),

            // Translation step 3
            new InstantCommand(this::alignReset),
            new WaitForTagCommand(pVision, this::passDesiredTarget, 2.0, (seen) -> activeTarget = seen),
            new InstantCommand(this::alignTranslation),
            new DriveDistanceAndHeading(pSwerve, () -> drivePerIterOffset),

            // One more thing. Apply the offset and end stuff.
            new DriveDistanceAndHeading(pSwerve, () -> new Pose2d(offsetFromTarget, Rotation2d.kZero)),
            new InstantCommand(() -> onComplete(false)),

            // Runnable to invoke on completion of the command.
            // Sophia's code again, just condensed a little.
            new InstantCommand(() -> { if (runOnComplete.isPresent()) runOnComplete.get().run(); })
        );

        SmartDashboard.putBoolean("Commands/VisionAlignCommand/Ready", false);
    }

    @Override
    public void cancel()
    {
        onComplete(true);
        super.cancel();
    }

    private void alignInitialize()
    {
        // Initialize.
        activeTarget = null;
        oldTarget = null;
        translationIter = 0;
        rotationIter = 0;
        SmartDashboard.putBoolean("Commands/VisionAlignCommand/Ready", false);

        System.out.println("[ALIGN] Vision align to reef begun.");
    }
    private void alignReset()
    {
        // Reset the tag so we know if we hit it again.
        oldTarget = activeTarget;
        activeTarget = null;
    }

    private int passDesiredTarget()
    {
        if (oldTarget == null) return -1;
        else return oldTarget.fiducialId;
    }

    private void alignTranslation()
    {
        translationIter++;
        if (activeTarget == null)
        {
            // We couldn't spot a tag in a reasonable time,
            // quit now and don't move.
            System.out.printf("[ALIGN] Translation step %d has failed!\n", translationIter);
            drivePerIterOffset = Pose2d.kZero;
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
            drivePerIterOffset = Pose2d.kZero;
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

    private void onComplete(boolean cancelled)
    {
        if (cancelled) System.out.println("[ALIGN] Vision alignment has been halted!");
        else System.out.println("[ALIGN] Vision alignment completed.");
        SmartDashboard.putBoolean("Commands/VisionAlignCommand/Ready", !cancelled);
    }
}
