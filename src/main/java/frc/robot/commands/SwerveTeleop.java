// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import static frc.robot.subsystems.SwerveSubsystem.kMaxSpeedMS;
import static frc.robot.subsystems.SwerveSubsystem.kMetersPerSecondToRadiansPerSecond;

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

    // FARIS_UPDATE_SWERVE
    private static final double slow_down = 0.1;

    // FARIS_UPDATE_SWERVE
    private final Supplier<Boolean> isTheBButtonHeld;

    /**
     * Creates a new ExampleCommand.
     *
     * @param pSwerveSubsystem The subsystem used by this command.
     */
    public SwerveTeleop(Supplier<Double> pFwdInput,
            Supplier<Double> pSideInput,
            Supplier<Double> pTurnInput,
            Supplier<Boolean> pResetHeadingInput,
            // FARIS_UPDATE_SWERVE
            Supplier<Boolean> pisTheBButtonHeld)
    {
        mFwdInput = pFwdInput;
        mSideInput = pSideInput;
        mTurnInput = pTurnInput;
        mResetHeadingInput = pResetHeadingInput;
        mSwerveSubsystem = SwerveSubsystem.kInstance;
        // FARIS_UPDATE_SWERVE
        isTheBButtonHeld = pisTheBButtonHeld;

        mChassisHeadingDegrees = mSwerveSubsystem.getHeadingDegrees();
        mFieldHeadingDegrees = 0;

        mGyroController.enableContinuousInput(0, 360);

        addRequirements(mSwerveSubsystem);
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

        // FARIS_UPDATE_SWERVE
        if (isTheBButtonHeld.get())
        {
            updatedFwdSpeedMS *= slow_down;
            updatedSideSpeedMS *= slow_down;
            updatedTurnSpeedRadS *= slow_down;

            mSwerveSubsystem.driveRobotOriented(new ChassisSpeeds
            (
                updatedFwdSpeedMS,
                updatedSideSpeedMS,
                updatedTurnSpeedRadS
            ));
        }

        else
        {
            mSwerveSubsystem.driveFieldOriented(new ChassisSpeeds(
                updatedFwdSpeedMS,
                updatedSideSpeedMS,
                updatedTurnSpeedRadS), 
            mFieldHeadingDegrees);
        }
        
        if (mResetHeadingInput.get())
        {
            resetFieldHeading();
        }

        SmartDashboard.putString("Swerve pose: ", mSwerveSubsystem.getPose().toString());
        SmartDashboard.putNumber("Swerve rotation: ", mSwerveSubsystem.getPose().getRotation().getDegrees());

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
