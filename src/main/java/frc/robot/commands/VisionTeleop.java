package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.objectmodels.ReefEndTarget;
import frc.robot.subsystems.ElevatorCarriageSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class VisionTeleop {
    
    private final static SwerveSubsystem mSwerveSubsystem = SwerveSubsystem.kInstance;
    private final static VisionSubsystem mVisionSubsystem = VisionSubsystem.kInstance;
    private final static ElevatorCarriageSubsystem mElevatorCarriageSubsystem = ElevatorCarriageSubsystem.kInstance;
    private final static VisionAlign mVisionAlign = new VisionAlign(mSwerveSubsystem, mVisionSubsystem);

    private final static Set<Subsystem> mRequirements = Set.of(mSwerveSubsystem, mVisionSubsystem);

    public static void setup(XboxController driver)
    {
        // Aligns to the nearest reef when X is pressed
        Trigger visionTrigger = new Trigger(driver::getXButton);
        visionTrigger.whileTrue(new DeferredCommand(() ->
            mVisionAlign.alignToNearestReef(() -> driver.getLeftTriggerAxis() > 0.1, () -> driver.getRightTriggerAxis() > 0.1, mElevatorCarriageSubsystem::getDesiredPreset),
            mRequirements)
        );

        // Aligns to the right loading station when right back button is pressed
        Trigger rightMatchLoadingTrigger = new Trigger(driver::getRightStickButton);
        rightMatchLoadingTrigger.whileTrue(new DeferredCommand(() ->
            mVisionAlign.alignToRightMatchLoadingStation(),
            mRequirements)
        );

        // Aligns to the left loading station when left back button is pressed
        Trigger leftMatchLoadingTrigger = new Trigger(driver::getLeftStickButton);
        leftMatchLoadingTrigger.whileTrue(new DeferredCommand(() ->
            mVisionAlign.alignToLeftMatchLoadingStation(),
            mRequirements)
        );

        // Aligns to the far left side of the reef from field-oriented view
        POVButton farLeftButton = new POVButton(driver, getDPADAngle(DPADAngle.TopLeft));
        farLeftButton.whileTrue(getReefCommand(ReefEndTarget.FarLeft, driver));

        // Aligns to the far middle side of the reef from field-oriented view
        POVButton farMiddleButton = new POVButton(driver, getDPADAngle(DPADAngle.TopMiddle));
        farMiddleButton.whileTrue(getReefCommand(ReefEndTarget.FarMiddle, driver));

        // Aligns to the far right side of the reef from field-oriented view
        POVButton farRightButton = new POVButton(driver, getDPADAngle(DPADAngle.TopRight));
        farRightButton.whileTrue(getReefCommand(ReefEndTarget.FarRight, driver));

        // Aligns to the near left side of the reef from field-oriented view 
        POVButton nearLeftButton = new POVButton(driver, getDPADAngle(DPADAngle.LowLeft));
        nearLeftButton.whileTrue(getReefCommand(ReefEndTarget.NearLeft, driver));

        // Aligns to the near middle side of the reef from field-oriented view
        POVButton nearMiddleButton = new POVButton(driver, getDPADAngle(DPADAngle.LowMiddle));
        nearMiddleButton.whileTrue(getReefCommand(ReefEndTarget.NearMiddle, driver));

        // Aligns to the near right side of the reef from field-oriented view
        POVButton nearRightButton = new POVButton(driver, getDPADAngle(DPADAngle.LowRight));
        nearRightButton.whileTrue(getReefCommand(ReefEndTarget.NearRight, driver));
    }

    // Gets a deferred command to align to a reef face because drive pose needs to reset when the command is scheduled.
    private static DeferredCommand getReefCommand(ReefEndTarget target, XboxController controller)
    {
        // Uses left and right triggers for left and right align
        // Checks for the current preset
        return new DeferredCommand(() ->
        mVisionAlign.alignToReef(target, () -> controller.getLeftTriggerAxis() > 0.1, () -> controller.getRightTriggerAxis() > 0.1, mElevatorCarriageSubsystem::getDesiredPreset),
        mRequirements);
    }

    private static int getDPADAngle(DPADAngle angle)
    {
        // Gets the correct DPAD angle
        switch (angle)
        {
            case TopLeft: return 315;
            case TopMiddle: return 0;
            case TopRight: return 45;
            case LowLeft: return 225;
            case LowMiddle: return 180;
            case LowRight: return 135;
            default: return 0;
        }
    }

    private enum DPADAngle {
        TopLeft,
        TopMiddle,
        TopRight,
        LowLeft,
        LowMiddle,
        LowRight
    }
}
