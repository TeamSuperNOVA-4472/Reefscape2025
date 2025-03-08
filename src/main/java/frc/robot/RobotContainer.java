// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlignToReef;
import frc.robot.commands.CloseUpOnReef;
import frc.robot.commands.DoTheThingCommand;
import frc.robot.commands.SwerveTeleop;
import frc.robot.commands.AlignToReef.EndTarget;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.Set;

import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
    private boolean isFirst;

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
        mVisionSubsystem = new VisionSubsystem(mSwerveSubsystem);

        isFirst = true;

        // Initialize commands.
        mSwerveTeleop = new SwerveTeleop(
            () -> mFwdLimiter.calculate(OperatorConstants.getControllerProfileValue(-mDriver.getLeftY())),
            () -> mSideLimiter.calculate(OperatorConstants.getControllerProfileValue(-mDriver.getLeftX())),
            () -> mTurnLimiter.calculate(OperatorConstants.getControllerProfileValue(-mDriver.getRightX())),
            mDriver::getAButton,
            mDriver::getYButton,
            mDriver::getBButton,
            mSwerveSubsystem,
            mVisionSubsystem);

        // Configure subsystems.
        mSwerveSubsystem.setDefaultCommand(mSwerveTeleop);
        mVisionSubsystem.addMeasurementListener((EstimatedRobotPose newVisionPose) -> {
            // Update the swerve's odometry with the new vision estimate.
            /*Pose2d vision = newVisionPose.estimatedPose.toPose2d();
            Pose2d current = mSwerveSubsystem.getPose();

            Pose2d measurement = new Pose2d(vision.getTranslation(), current.getRotation());*/
            mSwerveSubsystem.addVisionMeasurement(newVisionPose.estimatedPose.toPose2d(), newVisionPose.timestampSeconds);
            isFirst = false;
        });

        Trigger visionTrigger = new Trigger(mDriver::getXButton);
        visionTrigger.whileTrue(new DeferredCommand(() -> 
            new AlignToReef(
                mSwerveSubsystem, 
                mVisionSubsystem,
                AlignToReef.EndTarget.FAR_RIGHT), 
            Set.of(mSwerveSubsystem, mVisionSubsystem))
        );

        Trigger driveTrigger = new Trigger(mDriver::getRightBumperButton);
        driveTrigger.whileTrue(new DeferredCommand(() ->
            new CloseUpOnReef(mSwerveSubsystem, new Pose2d(15, 3, new Rotation2d())),
            Set.of(mSwerveSubsystem))
        );

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
        NamedCommands.registerCommand("VisionAlign", new DeferredCommand(() -> 
        new AlignToReef(
            mSwerveSubsystem, 
            mVisionSubsystem,
            AlignToReef.EndTarget.FAR_RIGHT), 
            Set.of(mSwerveSubsystem, mVisionSubsystem)));

        // TODO: DEBUG THING, PLEASE REMOVE
        new EventTrigger("TheEvent").onTrue(
                new InstantCommand(() -> System.out.println("The Event has triggered")));

        autoChooser.addOption("DriveDoTheThingDriveBack", new PathPlannerAuto("DriveDoTheThingDriveBack"));
        autoChooser.addOption("ReefScoreTopLeft", new PathPlannerAuto("ReefScoreTopLeft"));
        autoChooser.addOption("MidRightReefScore", new PathPlannerAuto("MidRightReefScore"));
        autoChooser.addOption("RightReefScoreToAlgae", new PathPlannerAuto("RightReefScoreToAlgae"));
        autoChooser.addOption("AmbitiousTopLeftScore", new PathPlannerAuto("AmbitiousTopLeftScore"));
        autoChooser.addOption("ReefScoreBottomRight", new PathPlannerAuto("ReefScoreBottomRight"));

        autoChooser.addOption("VisionTest", new PathPlannerAuto("VisionTest"));
        autoChooser.addOption("Tag Check", new PathPlannerAuto("Tag Check"));


        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    // Specify which command will be used as the autonomous command.
    public Command getAutonomousCommand()
    {
        return autoChooser.getSelected();
    }
}
