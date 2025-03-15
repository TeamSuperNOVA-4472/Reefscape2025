// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.parser.SwerveParser;

public class SwerveSubsystem extends SubsystemBase
{
    public static final SwerveSubsystem kInstance = new SwerveSubsystem();

    public static final double kMaxSpeedMS = 4.5;
    public static final double kMetersPerInch = Units.inchesToMeters(1);
    public static final double kSwerveLocYInches = 7.5;
    public static final double kSwerveLocXInches = 7;
    public static final double kSwerveLocYMeters = kSwerveLocYInches * kMetersPerInch;
    public static final double kSwerveLocXMeters = kSwerveLocXInches * kMetersPerInch;
    public static final double kSwerveRadiusInches = Math.sqrt(Math.pow(kSwerveLocXInches, 2) + Math.pow(kSwerveLocYInches, 2));
    public static final double kSwerveCircumferenceMeters = 2 * Math.PI * kSwerveRadiusInches * kMetersPerInch; // 1.6372863352652361048052029816421
    public static final double kMetersPerSecondToRadiansPerSecond = (2 * Math.PI) / kSwerveCircumferenceMeters;
    public static final double kS = 0.14557; //BL: 0.11472 //BR: 0.16985 //FL: 0.15312 //FR: 0.14459
    public static final double kV = 2.113175; //BL: 2.0722 //BR: 2.1609 //FL: 2.1133 //FR: 2.1063
    public static final double kA = 0.18558; //BL: 0.24512 //BR: 0.13383 //FL: 0.15566 //FR: 0.20771

    private final SwerveDrive mSwerveDrive;
    private final Command mSwerveSysID;

    private static SwerveDrive readSwerveConfig()
    {
        SwerveDrive swerveDrive = null;
        File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
        try
        {
            swerveDrive = new SwerveParser(swerveJsonDirectory)
                    .createSwerveDrive(kMaxSpeedMS);
            swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(kS, kV,
            kA));
        }
        catch (IOException e)
        {
            throw new RuntimeException(e);
        }
        return swerveDrive;
    }

    private static void configAutoBuilder(SwerveSubsystem pSwerveSubsystem)
    {
        // Load the RobotConfig from the GUI settings. You should probably
        // store this in your Constants file
        RobotConfig config;
        try
        {
            config = RobotConfig.fromGUISettings();

            // Configure AutoBuilder last
            AutoBuilder.configure(
                    pSwerveSubsystem::getPose, // Robot pose supplier
                    pSwerveSubsystem::resetOdometry, // Method to reset odometry (will be called if your auto has a
                                                     // starting pose)
                    pSwerveSubsystem::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    (speeds, feedforwards) -> pSwerveSubsystem.driveRobotOriented(speeds), // Method that will drive the
                                                                                           // robot given ROBOT RELATIVE
                                                                                           // ChassisSpeeds. Also
                                                                                           // optionally outputs
                                                                                           // individual module
                                                                                           // feedforwards
                    new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller
                                                    // for holonomic drive trains
                            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                    ),
                    config, // The robot configuration
                    () -> {
                        // Boolean supplier that controls when the path will be mirrored for the red
                        // alliance
                        // This will flip the path being followed to the red side of the field.
                        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    pSwerveSubsystem // Reference to this subsystem to set requirements
            );
        }
        catch (Exception e)
        {
            // Handle exception as needed
            e.printStackTrace();
        }
    }

    /** Creates a new ExampleSubsystem. */
    private SwerveSubsystem()
    {
        //SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        mSwerveDrive = readSwerveConfig();
        mSwerveDrive.setHeadingCorrection(false);
        mSwerveSysID = SwerveDriveTest.generateSysIdCommand(
            SwerveDriveTest.setDriveSysIdRoutine(
                new Config(),
                this,
                mSwerveDrive,
                12,
                true),
                3.0,
                5,
                3);
        configAutoBuilder(this);
    }

    public void addVisionMeasurement(Pose2d visionPose, double timestamp)
    {
        mSwerveDrive.addVisionMeasurement(visionPose, timestamp);
    }

    // Drive in some direction with its reference point set to the field itself.
    public void driveFieldOriented(ChassisSpeeds pVelocity, double pFieldHeadingDegrees)
    {
        ChassisSpeeds fieldOrientedVelocity =
            ChassisSpeeds.fromFieldRelativeSpeeds(
                pVelocity,
                Rotation2d.fromDegrees(getGyroDegrees()).minus(Rotation2d.fromDegrees(pFieldHeadingDegrees)));
            mSwerveDrive.drive(fieldOrientedVelocity);
    }


    public void driveFieldOriented(ChassisSpeeds pVelocity) {
        mSwerveDrive.driveFieldOriented(pVelocity);
    }

    public void driveRobotOriented(ChassisSpeeds pVelocity)
    {
        mSwerveDrive.drive(pVelocity);
    }

    public void driveTranslation(Translation2d pTranslation)
    {
        mSwerveDrive.drive(pTranslation, 0, true, false);
    }

    public void resetOdometry(Pose2d pPose)
    {
        mSwerveDrive.setGyro(new Rotation3d(0, 0, pPose.getRotation().getRadians()));
        mSwerveDrive.resetOdometry(pPose);
    }

    public Pose2d getPose()
    {
        return mSwerveDrive.getPose();
    }

    public double getHeadingDegrees()
    {
        return mSwerveDrive.getPose().getRotation().getDegrees();
    }

    /**
     * @return The current heading read from the gyroscope in degrees
     */
    public double getGyroDegrees() {
        double gyroReading = Rotation2d.fromRadians(mSwerveDrive.getGyro().getRotation3d().getZ()).getDegrees();
        return ((gyroReading % 360) + 360) % 360;
    }

    public ChassisSpeeds getRobotRelativeSpeeds()
    {
        return mSwerveDrive.getRobotVelocity();
    }

    public Command getSysIDCommand() {
        return mSwerveSysID;
    }
}
