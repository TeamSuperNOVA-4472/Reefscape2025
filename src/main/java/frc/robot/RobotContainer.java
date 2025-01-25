// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DoTheThingCommand;
import frc.robot.commands.SwerveTeleop;
import frc.robot.objectmodels.VisionPoseInfo;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

// This class is where subsystems and other robot parts are declared.
// IF A SUBSYSTEM IS NOT IN HERE, IT WILL NOT RUN!
public class RobotContainer
{
    // Ports go here:
    public static final int kDriverPort = 0;

    // Subsystems go here:
    private LightsSubsystem mLightsSubsystem;
    private VisionSubsystem mVisionSubsystem;
    private final SwerveSubsystem mSwerveSubsystem;

    // Controllers go here:
    private final XboxController mDriver;

    // Commands go here:
    private final SwerveTeleop mSwerveTeleop;

    // Extras:
    private final SendableChooser<Command> autoChooser;

    // TODO: I think these should be defined and handled by the swerve subsystem or command, not here.
    private final SlewRateLimiter mFwdLimiter = new SlewRateLimiter(1.0);
    private final SlewRateLimiter mSideLimiter = new SlewRateLimiter(1.0);
    private final SlewRateLimiter mTurnLimiter = new SlewRateLimiter(1.0);

    public RobotContainer()
    {        
        // Initialize controllers.
        mDriver = new XboxController(kDriverPort);

        // Initialize subsystems.
        mLightsSubsystem = new LightsSubsystem();
        mSwerveSubsystem = new SwerveSubsystem();
        mVisionSubsystem = new VisionSubsystem();

        // Initialize commands.
        mSwerveTeleop = new SwerveTeleop(
            () -> mFwdLimiter.calculate(OperatorConstants.getControllerProfileValue(-mDriver.getLeftY())),
            () -> mSideLimiter.calculate(OperatorConstants.getControllerProfileValue(-mDriver.getLeftX())),
            () -> mTurnLimiter.calculate(OperatorConstants.getControllerProfileValue(-mDriver.getRightX())),
            mDriver::getAButton,
            mSwerveSubsystem);

        // Configure subsystems.
        mSwerveSubsystem.setDefaultCommand(mSwerveTeleop);
        mVisionSubsystem.addMeasurementListener((VisionPoseInfo newVisionPose) -> {
            // Update the swerve's odometry with the new vision estimate.
            mSwerveSubsystem.addVisionMeasurement(newVisionPose.getPose(),
                                                  newVisionPose.getTimestamp());
        });

        // Configure other things.
        autoChooser = AutoBuilder.buildAutoChooser();

        // Register named commands.
        // TODO: Some of these are temporary things.
        NamedCommands.registerCommand("DoTheThingCommand", new DoTheThingCommand());
        NamedCommands.registerCommand("ScoreLevel4Right", new DoTheThingCommand());
        NamedCommands.registerCommand("ScoreLevel4Left", new DoTheThingCommand());
        NamedCommands.registerCommand("ScoreLevel3Right", new DoTheThingCommand());
        NamedCommands.registerCommand("IntakeCoral", new DoTheThingCommand());
        NamedCommands.registerCommand("GrabAlgae", new DoTheThingCommand());
        NamedCommands.registerCommand("ScoreAlgae", new DoTheThingCommand());

        // TODO: DEBUG THING, PLEASE REMOVE
        new EventTrigger("TheEvent").onTrue(
                new InstantCommand(() -> System.out.println("The Event has triggered")));

        autoChooser.addOption("DriveDoTheThingDriveBack", new PathPlannerAuto("DriveDoTheThingDriveBack"));
        autoChooser.addOption("ReefScoreTopLeft", new PathPlannerAuto("ReefScoreTopLeft"));
        autoChooser.addOption("MidRightReefScore", new PathPlannerAuto("MidRightReefScore"));
        autoChooser.addOption("RightReefScoreToAlgae", new PathPlannerAuto("RightReefScoreToAlgae"));
        autoChooser.addOption("AmbitiousTopLeftScore", new PathPlannerAuto("AmbitiousTopLeftScore"));
        autoChooser.addOption("ReefScoreBottomRight", new PathPlannerAuto("ReefScoreBottomRight"));

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    // Specify which command will be used as the autonomous command.
    public Command getAutonomousCommand()
    {
        return autoChooser.getSelected();
    }
}
