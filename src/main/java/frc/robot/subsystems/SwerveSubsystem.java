// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
// import com.pathplanner.lib.util.PIDConstants;
// import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import static frc.robot.Constants.SwerveConstants.*;


public class SwerveSubsystem extends SubsystemBase {

  private final SwerveDrive mSwerveDrive;

  private static SwerveDrive readSwerveConfig() {
    SwerveDrive swerveDrive = null;
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
    try {
      swerveDrive = new SwerveParser(swerveJsonDirectory)
        .createSwerveDrive(kMaxSpeedMS);
      swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
    return swerveDrive;
  }

  /** Creates a new ExampleSubsystem. */
  public SwerveSubsystem() {
    //SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    mSwerveDrive = readSwerveConfig();
    mSwerveDrive.setHeadingCorrection(false);
  }

  private boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
  }

  // Drive in some direction with its reference point set to the field itself.
  public void driveFieldOriented(ChassisSpeeds pVelocity)
  {
      if(isRedAlliance()) {
        ChassisSpeeds fieldOrientedVelocity =
          ChassisSpeeds.fromFieldRelativeSpeeds(
            pVelocity,
            mSwerveDrive.getYaw().plus(Rotation2d.fromRadians(Math.PI)));
        mSwerveDrive.drive(fieldOrientedVelocity);
      }
      else {
        mSwerveDrive.driveFieldOriented(pVelocity);
      }
  }

  public void driveRobotOriented(ChassisSpeeds pVelocity) {
    mSwerveDrive.drive(pVelocity);
  }

  public void resetOdometry(Pose2d pPose) {
    mSwerveDrive.setGyro(new Rotation3d(0, 0, pPose.getRotation().getRadians()));
    mSwerveDrive.resetOdometry(pPose);
  }

  public void resetHeading() {
    Rotation2d newHeading = Rotation2d.fromRadians(0);
    if(isRedAlliance()) {
      newHeading = Rotation2d.fromRadians(Math.PI);
    }
    resetOdometry(new Pose2d(getPose().getTranslation(), newHeading));
  }

  public Pose2d getPose() {
    return mSwerveDrive.getPose();
  }

  public double getHeadingDegrees() {
    return mSwerveDrive.getPose().getRotation().getDegrees();
  }

}
