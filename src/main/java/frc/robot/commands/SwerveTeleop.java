// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.subsystems.SwerveSubsystem.*; // For constants.

// TODO: Comments need to be added or replaced here.
// TODO: I also disagree with the use of Suppliers here, maybe discuss that next time.
//       Is this mostly last year's code?
public class SwerveTeleop extends Command
{
    private final Supplier<Double> mFwdInput;
    private final Supplier<Double> mSideInput;
    private final Supplier<Double> mTurnInput;
    private final Supplier<Boolean> mResetHeadingInput;
    private final Supplier<Boolean> mLeftButton;
    private final Supplier<Boolean> mRightButton;
    private final SwerveSubsystem mSwerveSubsystem;
    private final VisionSubsystem mVisionSubsystem;
    private final Trigger mVisionAlignTrigger;

    private final PIDController mGyroController = new PIDController(0.05, 0, 0.0005);
    private double mTargetHeading;

    /**
     * Creates a new ExampleCommand.
     *
     * @param pSwerveSubsystem The subsystem used by this command.
     */
    public SwerveTeleop(Supplier<Double> pFwdInput,
            Supplier<Double> pSideInput,
            Supplier<Double> pTurnInput,
            Supplier<Boolean> pResetHeadingInput,
            Supplier<Boolean> pLeftButton,
            Supplier<Boolean> pRightButton,
            SwerveSubsystem pSwerveSubsystem,
            VisionSubsystem pVisionSubsystem)
    {
        mFwdInput = pFwdInput;
        mSideInput = pSideInput;
        mTurnInput = pTurnInput;
        mResetHeadingInput = pResetHeadingInput;
        mLeftButton = pLeftButton;
        mRightButton = pRightButton;
        mSwerveSubsystem = pSwerveSubsystem;
        mVisionSubsystem = pVisionSubsystem;
        mVisionAlignTrigger = new Trigger(pLeftButton::get);
        mVisionAlignTrigger.whileTrue(new VisionAlignCommand(mSwerveSubsystem, mVisionSubsystem, Translation2d.kZero));

        mTargetHeading = mSwerveSubsystem.getHeadingDegrees();

        mGyroController.enableContinuousInput(0, 360);

        addRequirements(pSwerveSubsystem);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        SmartDashboard.putNumber("Swerve Rotation: ", mSwerveSubsystem.getPose().getRotation().getDegrees());
        double updatedFwdSpeedMS = mFwdInput.get() * kMaxSpeedMS;
        double updatedSideSpeedMS = mSideInput.get() * kMaxSpeedMS;
        double updatedTurnSpeedRadS = mTurnInput.get() * kMetersPerSecondToRadiansPerSecond * kMaxSpeedMS;

        if (updatedTurnSpeedRadS == 0.0 && (updatedFwdSpeedMS != 0 || updatedSideSpeedMS != 0))
        {
            updatedTurnSpeedRadS = mGyroController.calculate(mSwerveSubsystem.getHeadingDegrees(), mTargetHeading);
        }
        else
        {
            mTargetHeading = mSwerveSubsystem.getHeadingDegrees();
        }

        ChassisSpeeds updatedSpeeds = new ChassisSpeeds(
                updatedFwdSpeedMS,
                updatedSideSpeedMS,
                updatedTurnSpeedRadS);
        mSwerveSubsystem.driveFieldOriented(updatedSpeeds);

        if (mResetHeadingInput.get())
        {
            mSwerveSubsystem.resetHeading();
        }

    }
}
