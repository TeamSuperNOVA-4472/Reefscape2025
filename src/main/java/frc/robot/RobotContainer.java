// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DoTheThingCommand;
import frc.robot.commands.ElevatorCarriageTeleop;
import frc.robot.commands.IntakeTeleop;
import frc.robot.commands.MoveCarriageToPresetCommand;
import frc.robot.commands.MoveToLevelCommand;
import frc.robot.commands.SwerveTeleop;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.commands.tester.ElevatorTester;
import frc.robot.objectmodels.IntakePresets;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevatorCarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import frc.robot.commands.MoveToLevelCommand;

import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// This class is where subsystems and other robot parts are declared.
// IF A SUBSYSTEM IS NOT IN HERE, IT WILL NOT RUN!
public class RobotContainer
{
    // Ports go here:
    public static final int kDriverPort = 0;
    public static final int kPartnerPort = 1;

    // Subsystems go here:
    private LightsSubsystem mLightsSubsystem;
    private VisionSubsystem mVisionSubsystem;
    private final CarriageSubsystem mCarriageSubsystem;
    private final ClimbSubsystem mClimbSubsystem;
    private final ElevatorCarriageSubsystem mElevatorCarriageSubsystem;
    private final ElevatorSubsystem mElevatorSubsystem;
    private final IntakeSubsystem mIntakeSubsystem;
    private final SwerveSubsystem mSwerveSubsystem;

    // Controllers go here:
    private final XboxController mDriver;

    private final XboxController mPartner;
    
    // Commands go here:
    private final SwerveTeleop mSwerveTeleop;
    private final ElevatorCarriageTeleop mElevatorCarriageTeleop;
    private final IntakeTeleop mIntakeTeleop;

    // TODO: Remove tester commands when robot is properly programmed
    //private final ElevatorTester mElevatorTester;

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
        mPartner = new XboxController(kPartnerPort);

        // Initialize subsystems.
        mLightsSubsystem = new LightsSubsystem();
        mSwerveSubsystem = new SwerveSubsystem();
        mVisionSubsystem = new VisionSubsystem(mSwerveSubsystem);
        mCarriageSubsystem = new CarriageSubsystem();
        mClimbSubsystem = new ClimbSubsystem();
        mElevatorSubsystem = new ElevatorSubsystem();
        mElevatorCarriageSubsystem = new ElevatorCarriageSubsystem(mElevatorSubsystem, mCarriageSubsystem);
        mIntakeSubsystem = new IntakeSubsystem();
        

        // Initialize commands.
        mSwerveTeleop = new SwerveTeleop(
            () -> mFwdLimiter.calculate(OperatorConstants.getControllerProfileValue(-mDriver.getLeftY())),
            () -> mSideLimiter.calculate(OperatorConstants.getControllerProfileValue(-mDriver.getLeftX())),
            () -> mTurnLimiter.calculate(OperatorConstants.getControllerProfileValue(-mDriver.getRightX())),
            mDriver::getAButton,
            mDriver::getXButton,
            mDriver::getBButton,
            mSwerveSubsystem,
            mVisionSubsystem);

        // Configure subsystems.
        mSwerveSubsystem.setDefaultCommand(mSwerveTeleop);
        mVisionSubsystem.addMeasurementListener((EstimatedRobotPose newVisionPose) -> {
            // Update the swerve's odometry with the new vision estimate.
            mSwerveSubsystem.addVisionMeasurement(newVisionPose.estimatedPose.toPose2d(),
                                                  newVisionPose.timestampSeconds);
        });
        mElevatorCarriageTeleop = new ElevatorCarriageTeleop(mElevatorCarriageSubsystem, mDriver);
        mIntakeTeleop = new IntakeTeleop(mIntakeSubsystem, mDriver::getLeftBumperButton, mDriver::getRightBumperButton);

        // Configure subsystems.
        mSwerveSubsystem.setDefaultCommand(mSwerveTeleop);
        //mElevatorCarriageSubsystem.setDefaultCommand(mElevatorCarriageTeleop);
        //mIntakeSubsystem.setDefaultCommand(mElevatorCarriageTeleop);

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
        return new SequentialCommandGroup(new MoveToLevelCommand(mElevatorSubsystem, IntakePresets.kScoreL3));
    }
}
