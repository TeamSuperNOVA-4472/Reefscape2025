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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class SwerveSubsystem extends SubsystemBase
{
    public static final double kMaxSpeedMS = 4.5;
    public static final double kMetersPerInch = Units.inchesToMeters(1);
    public static final double kSwerveLocYInches = 7.5;
    public static final double kSwerveLocXInches = 7;
    public static final double kSwerveLocYMeters = kSwerveLocYInches * kMetersPerInch;
    public static final double kSwerveLocXMeters = kSwerveLocXInches * kMetersPerInch;
    public static final double kSwerveRadiusInches = Math.sqrt(Math.pow(kSwerveLocXInches, 2) + Math.pow(kSwerveLocYInches, 2));
    public static final double kSwerveCircumferenceMeters = 2 * Math.PI * kSwerveRadiusInches * kMetersPerInch; // 1.6372863352652361048052029816421
    public static final double kMetersPerSecondToRadiansPerSecond = (2 * Math.PI) / kSwerveCircumferenceMeters;
    public static final double kS = 0.267;
    public static final double kV = 2.65;
    public static final double kA = 0.239;

    private final SwerveDrive mSwerveDrive;

    private static SwerveDrive readSwerveConfig()
    {
        SwerveDrive swerveDrive = null;
        File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
        try
        {
            swerveDrive = new SwerveParser(swerveJsonDirectory)
                    .createSwerveDrive(kMaxSpeedMS);
            // swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(kS, kV,
            // kA));
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
    public SwerveSubsystem()
    {
        // SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        mSwerveDrive = readSwerveConfig();
        mSwerveDrive.setHeadingCorrection(false);
        configAutoBuilder(this);
    }

    public void addVisionMeasurement(Pose2d visionPose, double timestamp)
    {
        mSwerveDrive.addVisionMeasurement(visionPose, timestamp);
    }

    // TODO: This should be moved to RobotContainer or Robot, I think.
    private boolean isRedAlliance()
    {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent())
        {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }

    // Drive in some direction with its reference point set to the field itself.
    public void driveFieldOriented(ChassisSpeeds pVelocity)
    {
        if (isRedAlliance())
        {
            ChassisSpeeds fieldOrientedVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(
                    pVelocity,
                    mSwerveDrive.getYaw().plus(Rotation2d.fromRadians(Math.PI)));
            mSwerveDrive.drive(fieldOrientedVelocity);
        }
        else
        {
            mSwerveDrive.driveFieldOriented(pVelocity);
        }
    }

    public void driveRobotOriented(ChassisSpeeds pVelocity)
    {
        mSwerveDrive.drive(pVelocity);
    }

    public void resetOdometry(Pose2d pPose)
    {
        mSwerveDrive.setGyro(new Rotation3d(0, 0, pPose.getRotation().getRadians()));
        mSwerveDrive.resetOdometry(pPose);
    }

    public void resetHeading()
    {
        Rotation2d newHeading = Rotation2d.fromRadians(0);
        if (isRedAlliance())
        {
            newHeading = Rotation2d.fromRadians(Math.PI);
        }
        resetOdometry(new Pose2d(getPose().getTranslation(), newHeading));
    }

    public Pose2d getPose()
    {
        return mSwerveDrive.getPose();
    }

    public double getHeadingDegrees()
    {
        return mSwerveDrive.getPose().getRotation().getDegrees();
    }

    public ChassisSpeeds getRobotRelativeSpeeds()
    {
        return mSwerveDrive.getRobotVelocity();
    }
}
