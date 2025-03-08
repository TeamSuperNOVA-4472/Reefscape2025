// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystem;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.Constants.SwerveConstants.*;

/** An example command that uses an example subsystem. */
public class SwerveTeleop extends Command {

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
    SwerveSubsystem pSwerveSubsystem) {
  
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
  public void execute() {
  
    double updatedFwdSpeedMS = mFwdInput.get() * kMaxSpeedMS;
    double updatedSideSpeedMS = mSideInput.get() * kMaxSpeedMS;
    double updatedTurnSpeedRadS =
      mTurnInput.get() * kMetersPerSecondToRadiansPerSecond * kMaxSpeedMS;


    if(updatedTurnSpeedRadS == 0.0 && (updatedFwdSpeedMS != 0 || updatedSideSpeedMS != 0)) {
      updatedTurnSpeedRadS = mGyroController.calculate(mSwerveSubsystem.getGyroDegrees(), mChassisHeadingDegrees);
    } else {
      mChassisHeadingDegrees = mSwerveSubsystem.getGyroDegrees();
    }

    ChassisSpeeds updatedSpeeds = new ChassisSpeeds(
      updatedFwdSpeedMS,
      updatedSideSpeedMS, 
      updatedTurnSpeedRadS);
    mSwerveSubsystem.driveFieldOriented(updatedSpeeds, mFieldHeadingDegrees);

    if(mResetHeadingInput.get()) {
      resetFieldHeading();
    }
    SmartDashboard.putNumber("Field Heading", mFieldHeadingDegrees);
    SmartDashboard.putNumber("Chassis Heading", mChassisHeadingDegrees);
    SmartDashboard.putNumber("Odometry Heading", mSwerveSubsystem.getHeadingDegrees());
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
