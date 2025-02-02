// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

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
    private final Supplier<Boolean> mButton;
    private final SwerveSubsystem mSwerveSubsystem;
    private final VisionSubsystem mVisionSubsystem;
    private final VisionAlignCommand mAlign;

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
            Supplier<Boolean> pButton,
            SwerveSubsystem pSwerveSubsystem,
            VisionSubsystem pVisionSubsystem)
    {
        mFwdInput = pFwdInput;
        mSideInput = pSideInput;
        mTurnInput = pTurnInput;
        mResetHeadingInput = pResetHeadingInput;
        mButton = pButton;
        mSwerveSubsystem = pSwerveSubsystem;
        mVisionSubsystem = pVisionSubsystem;

        mTargetHeading = mSwerveSubsystem.getHeadingDegrees();

        mGyroController.enableContinuousInput(0, 360);

        mAlign = new VisionAlignCommand(pSwerveSubsystem, pVisionSubsystem, null);

        addRequirements(pSwerveSubsystem);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
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

        if (mButton.get())
        {
            if(!mAlign.isScheduled())
            {
                mAlign.schedule();
            }
        } else {
            mAlign.cancel();
        }
    }
}
