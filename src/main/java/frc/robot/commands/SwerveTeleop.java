// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

import static frc.robot.subsystems.SwerveSubsystem.*; // For constants.

// TODO: Comments need to be added or replaced here.
// TODO: I also disagree with the use of Suppliers here, maybe discuss that next time.
//       Is this mostly last year's code?
public class SwerveTeleop extends Command
{
    // TODO: how is this different from a clamp? Might want to consider doing that.
    public static final SlewRateLimiter mFwdLimiter = new SlewRateLimiter(1.0);
    public static final SlewRateLimiter mSideLimiter = new SlewRateLimiter(1.0);
    public static final SlewRateLimiter mTurnLimiter = new SlewRateLimiter(1.0);

    private final Supplier<Double> mFwdInput;
    private final Supplier<Double> mSideInput;
    private final Supplier<Double> mTurnInput;
    private final Supplier<Boolean> mResetHeadingInput;
    private final SwerveSubsystem mSwerveSubsystem;

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
            SwerveSubsystem pSwerveSubsystem)
    {
        mFwdInput = pFwdInput;
        mSideInput = pSideInput;
        mTurnInput = pTurnInput;
        mResetHeadingInput = pResetHeadingInput;
        mSwerveSubsystem = pSwerveSubsystem;

        mTargetHeading = mSwerveSubsystem.getHeadingDegrees();

        mGyroController.enableContinuousInput(0, 360);

        addRequirements(pSwerveSubsystem);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        double updatedFwdSpeedMS = mFwdLimiter.calculate(mFwdInput.get()) * kMaxSpeedMS;
        double updatedSideSpeedMS = mSideLimiter.calculate(mSideInput.get()) * kMaxSpeedMS;
        double updatedTurnSpeedRadS = mTurnLimiter.calculate(mTurnInput.get()) * kMetersPerSecondToRadiansPerSecond * kMaxSpeedMS;

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
