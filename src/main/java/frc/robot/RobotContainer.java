// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DoTheThingCommand;
import frc.robot.commands.SwerveTeleop;
import frc.robot.commands.Intake.AlgaeIntakePreset;
import frc.robot.commands.Intake.IntakeCoralTeleopCommand;
import frc.robot.commands.Intake.LoadCoral;
import frc.robot.commands.Intake.MoveCarriageToPresetCommand;
import frc.robot.commands.Intake.MoveToLevelCommand;
import frc.robot.commands.Intake.StowCarriagePosition;
import frc.robot.commands.tester.CarriageTester;
import frc.robot.commands.tester.ClimberTester;
import frc.robot.commands.tester.ElevatorTester;
import frc.robot.commands.tester.IntakeTester;
import frc.robot.objectmodels.IntakePreset;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevatorCarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

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
import edu.wpi.first.wpilibj2.command.button.Trigger;

// This class is where subsystems and other robot parts are declared.
// IF A SUBSYSTEM IS NOT IN HERE, IT WILL NOT RUN!
public class RobotContainer
{
    // Ports go here:
    public static final int kDriverPort = 0;
    public static final int kPartnerPort = 1;

    // Subsystems go here:
    private final CarriageSubsystem mCarriageSubsystem;
    //private final ElevatorCarriageSubsystem mElevatorCarriageSubsystem;
    private final ElevatorSubsystem mElevatorSubsystem;
    private final IntakeSubsystem mIntakeSubsystem;
    private LightsSubsystem mLightsSubsystem;
    private VisionSubsystem mVisionSubsystem;
    private final SwerveSubsystem mSwerveSubsystem;
    private final ClimbSubsystem mClimbSubsystem;

    // Controllers go here:
    private final XboxController mDriver;
    private final XboxController mPartner;
    
    // Commands go here:
    private final SwerveTeleop mSwerveTeleop;
    //private final ElevatorCarriageTeleop mElevatorCarriageTeleop;
    //private final IntakeTeleop mIntakeTeleop;

    // TODO: Remove tester commands when robot is properly programmed
    private final ElevatorTester mElevatorTester;
    private final CarriageTester mCarriageTester;
    private final IntakeTester mIntakeTester;
    //private final ClimberTester mClimberTester;


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
        mCarriageSubsystem = new CarriageSubsystem();
        mClimbSubsystem = new ClimbSubsystem();
        mElevatorSubsystem = new ElevatorSubsystem();
        //mElevatorCarriageSubsystem = new ElevatorCarriageSubsystem(mElevatorSubsystem, mCarriageSubsystem);
        mIntakeSubsystem = new IntakeSubsystem();
        
        mVisionSubsystem = new VisionSubsystem(mSwerveSubsystem);

        // Initialize commands.
        mSwerveTeleop = new SwerveTeleop(
            () -> mFwdLimiter.calculate(OperatorConstants.getControllerProfileValue(-mDriver.getLeftY())),
            () -> mSideLimiter.calculate(OperatorConstants.getControllerProfileValue(-mDriver.getLeftX())),
            () -> mTurnLimiter.calculate(OperatorConstants.getControllerProfileValue(-mDriver.getRightX())),
            mDriver::getLeftBumperButton,
            mDriver::getRightBumperButton,
            mDriver::getAButton,
            mSwerveSubsystem,
            mIntakeSubsystem,
            mVisionSubsystem);
        // mElevatorCarriageTeleop = new ElevatorCarriageTeleop(mElevatorCarriageSubsystem, mDriver);
        // mIntakeTeleop = new IntakeTeleop(mIntakeSubsystem, mDriver::getLeftBumperButton, mDriver::getRightBumperButton);

        Trigger level1 = new Trigger(mPartner::getAButton);
        level1.onTrue(new IntakeCoralTeleopCommand(mCarriageSubsystem, mElevatorSubsystem, mIntakeSubsystem, IntakePreset.kScoreL1));

        Trigger level2 = new Trigger(mPartner::getXButton);
        level2.onTrue(new IntakeCoralTeleopCommand(mCarriageSubsystem, mElevatorSubsystem, mIntakeSubsystem, IntakePreset.kScoreL2));

        Trigger level3 = new Trigger(mPartner::getBButton);
        level3.onTrue(new IntakeCoralTeleopCommand(mCarriageSubsystem, mElevatorSubsystem, mIntakeSubsystem, IntakePreset.kScoreL3));

        Trigger level4 = new Trigger(mPartner::getYButton);
        level4.onTrue(new IntakeCoralTeleopCommand(mCarriageSubsystem, mElevatorSubsystem, mIntakeSubsystem, IntakePreset.kScoreL4));

        Trigger carriage = new Trigger(mPartner::getLeftBumperButton);
        carriage.onTrue(new LoadCoral(mElevatorSubsystem, mCarriageSubsystem, mIntakeSubsystem));

        Trigger scoreTrigger = new Trigger(mPartner::getRightBumperButton);
        scoreTrigger.onTrue(new StowCarriagePosition(mCarriageSubsystem, mElevatorSubsystem)
        );

        /*Trigger l1Trigger = new Trigger(() -> mPartner.getPOV() == 90);
        l1Trigger.onTrue(new CoralL1Preset(mElevatorSubsystem, mCarriageSubsystem)
        );

        Trigger l2Trigger = new Trigger(() -> mPartner.getPOV() == 180);
        l2Trigger.onTrue(new CoralL2Preset(mElevatorSubsystem, mCarriageSubsystem)
        );
        
        Trigger l3Trigger = new Trigger(() -> mPartner.getPOV() == 270);
        l3Trigger.onTrue(new CoralL3Preset(mElevatorSubsystem, mCarriageSubsystem)
        );

        Trigger l4Trigger = new Trigger(() -> mPartner.getPOV() == 0);
        l4Trigger.onTrue(new CoralL4Preset(mElevatorSubsystem, mCarriageSubsystem)
        );

        Trigger algaeTrigger = new Trigger(() -> mPartner.getLeftTriggerAxis() > 0);
        algaeTrigger.onTrue(new AlgaeIntakePreset(mCarriageSubsystem)
        );*/


        // TODO: remove tester commands when robot is properly programmed
        mElevatorTester = new ElevatorTester(mElevatorSubsystem, () -> MathUtil.applyDeadband(-mPartner.getLeftY(), 0.1));
        
        mCarriageTester = new CarriageTester(() -> MathUtil.applyDeadband(mPartner.getRightX(), 0.1), () -> MathUtil.applyDeadband(mPartner.getRightY(), 0.1), mCarriageSubsystem);
        mIntakeTester = new IntakeTester(
            mPartner::getLeftBumperButton,
            () -> mPartner.getLeftTriggerAxis() > 0.5,
            mIntakeSubsystem);
        //mClimberTester = new ClimberTester(mClimbSubsystem, mDriver::getLeftBumperButton, mDriver::getRightBumperButton);
        // Configure subsystems
        mSwerveSubsystem.setDefaultCommand(mSwerveTeleop);
        // mElevatorCarriageSubsystem.setDefaultCommand(mElevatorCarriageTeleop);
        // mIntakeSubsystem.setDefaultCommand(mElevatorCarriageTeleop);

        // TODO: remove tester commands when robot is properly programmed
        mElevatorSubsystem.setDefaultCommand(mElevatorTester);
        mCarriageSubsystem.setDefaultCommand(mCarriageTester);
        mIntakeSubsystem.setDefaultCommand(mIntakeTester);

        mVisionSubsystem.addMeasurementListener((EstimatedRobotPose newVisionPose) -> {
            // Update the swerve's odometry with the new vision estimate.
            mSwerveSubsystem.addVisionMeasurement(newVisionPose.estimatedPose.toPose2d(),
                                                  newVisionPose.timestampSeconds);
        });

        //mClimbSubsystem.setDefaultCommand(mClimberTester);

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
