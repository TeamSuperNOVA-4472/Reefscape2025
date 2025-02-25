// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.VisionSubsystem;
import frc.robot.commands.DoTheThingCommand;
import frc.robot.commands.ElevatorCarriageTeleop;
import frc.robot.commands.IntakeTeleop;
import frc.robot.commands.MoveCarriageToPresetCommand;
import frc.robot.commands.MoveToLevelCommand;
import frc.robot.commands.SwerveTeleop;
import frc.robot.commands.VisionAlignCommand;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.commands.Presets.AlgaeBarge;
import frc.robot.commands.Presets.AlgaeIntakePreset;
import frc.robot.commands.Presets.AlgaeL2;
import frc.robot.commands.Presets.AlgaeL3;
import frc.robot.commands.Presets.AlgaeProcessor;
import frc.robot.commands.Presets.CoralL1Preset;
import frc.robot.commands.Presets.CoralL2Preset;
import frc.robot.commands.Presets.CoralL3Preset;
import frc.robot.commands.Presets.CoralL4Preset;
import frc.robot.commands.Presets.LoadCoral;
import frc.robot.commands.Presets.StowCarriagePosition;
import frc.robot.commands.autoCommands.ScoreLevel1;
import frc.robot.commands.autoCommands.ScoreLevel3;
import frc.robot.commands.tester.CarriageTester;
import frc.robot.commands.tester.ClimberTester;
import frc.robot.commands.tester.ElevatorTester;
import frc.robot.commands.tester.IntakeTester;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevatorCarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.OperatorConfig.weightJoystick;

// This class is where subsystems and other robot parts are declared.
// IF A SUBSYSTEM IS NOT IN HERE, IT WILL NOT RUN!
@SuppressWarnings("unused")
public class RobotContainer
{
    // Ports go here:
    public static final int kDriverPort = 0;
    public static final int kPartnerPort = 1;

    // Subsystems go here:
    public LightsSubsystem mLightsSubsystem;
    private VisionSubsystem mVisionSubsystem;
    private final CarriageSubsystem mCarriageSubsystem;
    //private final ElevatorCarriageSubsystem mElevatorCarriageSubsystem;
    private final ElevatorSubsystem mElevatorSubsystem;
    private final IntakeSubsystem mIntakeSubsystem;
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
    private final ClimberTester mClimberTester;


    // Extras:
    private final SendableChooser<Command> autoChooser;

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
        mElevatorSubsystem = new ElevatorSubsystem(mLightsSubsystem);
        mClimbSubsystem = new ClimbSubsystem();
        //mElevatorCarriageSubsystem = new ElevatorCarriageSubsystem(mElevatorSubsystem, mCarriageSubsystem);
        mIntakeSubsystem = new IntakeSubsystem();

        // Initialize commands.
        // TODO: Should weighting go here? Or in the command?
        //       Also, we should add a comment explaining why
        //       we need to invert these.
        mSwerveTeleop = new SwerveTeleop(
            weightJoystick(mDriver::getLeftY, true),
            weightJoystick(mDriver::getLeftX, true),
            weightJoystick(mDriver::getRightX, true),
            mDriver::getAButton,
            mDriver::getXButton,
            mDriver::getBButton,
            mSwerveSubsystem,
            mVisionSubsystem);
        // mElevatorCarriageTeleop = new ElevatorCarriageTeleop(mElevatorCarriageSubsystem, mDriver);
        // mIntakeTeleop = new IntakeTeleop(mIntakeSubsystem, mDriver::getLeftBumperButton, mDriver::getRightBumperButton);
        Trigger carriage = new Trigger(mPartner::getRightBumperButton);
        carriage.onTrue(
            new ConditionalCommand(
                new InstantCommand(),
                new LoadCoral(mElevatorSubsystem, mCarriageSubsystem), 
                () -> mIntakeSubsystem.hasAlgae()).andThen(new InstantCommand(() -> mIntakeSubsystem.intakeCoral()))
        );
        carriage.onFalse(new InstantCommand(() -> mIntakeSubsystem.stop()).andThen(new StowCarriagePosition(mCarriageSubsystem, mElevatorSubsystem)));

        Trigger scoreTrigger = new Trigger(() -> mPartner.getRightTriggerAxis() > 0);
        scoreTrigger.onTrue(
            new InstantCommand(() -> mIntakeSubsystem.outtakeCoral())
        );
        scoreTrigger.onFalse(new InstantCommand(() -> mIntakeSubsystem.stop()));

        Trigger l1Trigger = new Trigger(mPartner::getAButton);
        l1Trigger.onTrue(
            new CoralL1Preset(mElevatorSubsystem, mCarriageSubsystem, mIntakeSubsystem)
        );

        Trigger l2Trigger = new Trigger(mPartner::getXButton);
        l2Trigger.onTrue(
            new CoralL2Preset(mElevatorSubsystem, mCarriageSubsystem, mIntakeSubsystem)
        );
        
        Trigger l3Trigger = new Trigger(mPartner::getBButton);
        l3Trigger.onTrue( 
            new CoralL3Preset(mElevatorSubsystem, mCarriageSubsystem, mIntakeSubsystem)
        );

        Trigger l4Trigger = new Trigger(mPartner::getYButton);
        l4Trigger.onTrue(
            new CoralL4Preset(mElevatorSubsystem, mCarriageSubsystem, mIntakeSubsystem)
        );

        Trigger algaeTrigger = new Trigger(mPartner::getLeftBumperButton);
        algaeTrigger.onTrue(
            new AlgaeL2(mElevatorSubsystem, mCarriageSubsystem, mIntakeSubsystem)//.andThen(new InstantCommand(() -> mIntakeSubsystem.intakeAlgae()))
        );

        Trigger algaeHighTrigger = new Trigger(() -> mPartner.getLeftTriggerAxis() > 0);
        algaeHighTrigger.onTrue(
            new AlgaeL3(mElevatorSubsystem, mCarriageSubsystem, mIntakeSubsystem)//.andThen(new InstantCommand(() -> mIntakeSubsystem.intakeAlgae()))
        );

        Trigger algaeProcessTrigger = new Trigger(() -> mPartner.getPOV() == 180);
        algaeProcessTrigger.onTrue(
            new AlgaeProcessor(mElevatorSubsystem, mCarriageSubsystem, mIntakeSubsystem)
        );

        Trigger algaeOut = new Trigger(() -> mPartner.getPOV() == 0);
        algaeOut.onTrue(
            new InstantCommand(() -> mIntakeSubsystem.outtakeAlgae())
        );
        algaeOut.onFalse(new InstantCommand(() -> mIntakeSubsystem.stop()));

        // TODO: remove tester commands when robot is properly programmed
        mElevatorTester = new ElevatorTester(mElevatorSubsystem, () -> MathUtil.applyDeadband(-mPartner.getLeftY(), 0.1));
        mCarriageTester = new CarriageTester(() -> MathUtil.applyDeadband(mPartner.getRightX(), 0.1), () -> MathUtil.applyDeadband(mPartner.getRightY(), 0.1), mCarriageSubsystem);
        mIntakeTester = new IntakeTester(mPartner::getAButton, mPartner::getBButton, mPartner::getXButton, mPartner::getYButton, mIntakeSubsystem);
        mClimberTester = new ClimberTester(mClimbSubsystem, mDriver::getLeftBumperButton, mDriver::getRightBumperButton, mDriver::getLeftTriggerAxis);
        // Configure subsystems
        mSwerveSubsystem.setDefaultCommand(mSwerveTeleop);
        /*mVisionSubsystem.addMeasurementListener((EstimatedRobotPose newVisionPose) -> {
            // Update the swerve's odometry with the new vision estimate.
            mSwerveSubsystem.addVisionMeasurement(newVisionPose.estimatedPose.toPose2d(),
                                                  newVisionPose.timestampSeconds);
        });*/ // FIXME: This causes weird things.

        // mElevatorCarriageSubsystem.setDefaultCommand(mElevatorCarriageTeleop);
        // mIntakeSubsystem.setDefaultCommand(mElevatorCarriageTeleop);

        // TODO: remove tester commands when robot is properly programmed
        //mElevatorSubsystem.setDefaultCommand(mElevatorTester);
        //mCarriageSubsystem.setDefaultCommand(mCarriageTester);
        //mIntakeSubsystem.setDefaultCommand(mIntakeTester);
        //mClimbSubsystem.setDefaultCommand(mClimberTester);


        // Register named commands.
        // TODO: Some of these are temporary things.
        NamedCommands.registerCommand("ScoreLevel1", new ScoreLevel1(mElevatorSubsystem, mCarriageSubsystem, mIntakeSubsystem));

        // Configure other things.
        autoChooser = AutoBuilder.buildAutoChooser();

        // TODO: DEBUG THING, PLEASE REMOVE
        new EventTrigger("TheEvent").onTrue(
                new InstantCommand(() -> System.out.println("The Event has triggered")));

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    // Specify which command will be used as the autonomous command.
    public Command getAutonomousCommand()
    {
        return autoChooser.getSelected();
    }
}
