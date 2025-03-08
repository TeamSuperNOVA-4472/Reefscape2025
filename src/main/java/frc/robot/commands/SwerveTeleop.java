// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
    private double mChassisHeadingDegrees;
    private double mFieldHeadingDegrees;


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

        mChassisHeadingDegrees = mSwerveSubsystem.getHeadingDegrees();
        mFieldHeadingDegrees = 0;

        mGyroController.enableContinuousInput(0, 360);

        addRequirements(pSwerveSubsystem);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        SmartDashboard.putNumber("Swerve Rotation: ", mSwerveSubsystem.getPose().getRotation().getDegrees());
        double updatedFwdSpeedMS = mFwdLimiter.calculate(mFwdInput.get()) * kMaxSpeedMS;
        double updatedSideSpeedMS = mSideLimiter.calculate(mSideInput.get()) * kMaxSpeedMS;
        double updatedTurnSpeedRadS = mTurnLimiter.calculate(mTurnInput.get()) * kMetersPerSecondToRadiansPerSecond * kMaxSpeedMS;

        if (updatedTurnSpeedRadS == 0.0 && (updatedFwdSpeedMS != 0 || updatedSideSpeedMS != 0))
        {
            updatedTurnSpeedRadS = mGyroController.calculate(mSwerveSubsystem.getGyroDegrees(), mChassisHeadingDegrees);
        }
        else
        {
            mChassisHeadingDegrees = mSwerveSubsystem.getGyroDegrees();
        }

        ChassisSpeeds updatedSpeeds = new ChassisSpeeds(
                updatedFwdSpeedMS,
                updatedSideSpeedMS,
                updatedTurnSpeedRadS);
                mSwerveSubsystem.driveFieldOriented(updatedSpeeds, mFieldHeadingDegrees);

        if (mResetHeadingInput.get())
        {
            resetFieldHeading();
        }

    }

    private void resetFieldHeading() {
        mFieldHeadingDegrees = mSwerveSubsystem.getGyroDegrees();
        mChassisHeadingDegrees = mSwerveSubsystem.getGyroDegrees();
    }

    public void resetHeadingToAlliance() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if(alliance.isEmpty()) {
            return;
        }


        double targetOdomFieldHeading = 0;
        if(alliance.get() == Alliance.Red) {
            targetOdomFieldHeading = 180;
        }
        mFieldHeadingDegrees =
            mSwerveSubsystem.getGyroDegrees()
                - mSwerveSubsystem.getHeadingDegrees()
                + targetOdomFieldHeading;

        mChassisHeadingDegrees = mSwerveSubsystem.getGyroDegrees();
   }

}
