// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DoTheThingCommand;
import frc.robot.commands.ElevatorCarriageTeleop;
import frc.robot.commands.IntakeTeleop;
import frc.robot.commands.MoveCarriageToPresetCommand;
import frc.robot.commands.MoveToLevelCommand;
import frc.robot.commands.SwerveTeleop;
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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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
        mSwerveSubsystem = new SwerveSubsystem();
        mCarriageSubsystem = new CarriageSubsystem();
        mClimbSubsystem = new ClimbSubsystem();
        mElevatorSubsystem = new ElevatorSubsystem();
        //mElevatorCarriageSubsystem = new ElevatorCarriageSubsystem(mElevatorSubsystem, mCarriageSubsystem);
       mIntakeSubsystem = new IntakeSubsystem();
        

        // Initialize commands.
        mSwerveTeleop = new SwerveTeleop(
            () -> mFwdLimiter.calculate(OperatorConstants.getControllerProfileValue(-mDriver.getLeftY())),
            () -> mSideLimiter.calculate(OperatorConstants.getControllerProfileValue(-mDriver.getLeftX())),
            () -> mTurnLimiter.calculate(OperatorConstants.getControllerProfileValue(-mDriver.getRightX())),
            mDriver::getAButton,
            mSwerveSubsystem);
        // mElevatorCarriageTeleop = new ElevatorCarriageTeleop(mElevatorCarriageSubsystem, mDriver);
        // mIntakeTeleop = new IntakeTeleop(mIntakeSubsystem, mDriver::getLeftBumperButton, mDriver::getRightBumperButton);
        Trigger carriage = new Trigger(mPartner::getLeftBumperButton);
        carriage.onTrue(new ConditionalCommand(new InstantCommand(), new LoadCoral(mElevatorSubsystem, mCarriageSubsystem), () -> mIntakeSubsystem.hasAlgae())
        );

        Trigger scoreTrigger = new Trigger(mPartner::getRightBumperButton);
        scoreTrigger.onTrue(new ConditionalCommand(new InstantCommand(), new StowCarriagePosition(mCarriageSubsystem, mElevatorSubsystem), () -> mIntakeSubsystem.hasAlgae())
        );

        Trigger l1Trigger = new Trigger(() -> mPartner.getPOV() == 90);
        l1Trigger.onTrue(new ConditionalCommand(new AlgaeProcessor(mElevatorSubsystem, mCarriageSubsystem), new CoralL1Preset(mElevatorSubsystem, mCarriageSubsystem), () -> mCarriageSubsystem.getAlgaeMode())
        );

        Trigger l2Trigger = new Trigger(() -> mPartner.getPOV() == 180);
        l2Trigger.onTrue(new ConditionalCommand(new AlgaeL2(mElevatorSubsystem, mCarriageSubsystem), new CoralL2Preset(mElevatorSubsystem, mCarriageSubsystem), () -> mCarriageSubsystem.getAlgaeMode())
        );
        
        Trigger l3Trigger = new Trigger(() -> mPartner.getPOV() == 270);
        l3Trigger.onTrue(new ConditionalCommand(new AlgaeL3(mElevatorSubsystem, mCarriageSubsystem), new CoralL3Preset(mElevatorSubsystem, mCarriageSubsystem), () -> mCarriageSubsystem.getAlgaeMode())
        );

        Trigger l4Trigger = new Trigger(() -> mPartner.getPOV() == 0);
        l4Trigger.onTrue(new ConditionalCommand(new AlgaeBarge(mElevatorSubsystem, mCarriageSubsystem), new CoralL4Preset(mElevatorSubsystem, mCarriageSubsystem), () -> mCarriageSubsystem.getAlgaeMode())
        );

        Trigger algaeTrigger = new Trigger(() -> mPartner.getLeftTriggerAxis() > 0);
        algaeTrigger.onTrue(new AlgaeIntakePreset(mCarriageSubsystem, mElevatorSubsystem)
        );

        // TODO: remove tester commands when robot is properly programmed
        mElevatorTester = new ElevatorTester(mElevatorSubsystem, () -> MathUtil.applyDeadband(-mPartner.getLeftY(), 0.1));
        
        mCarriageTester = new CarriageTester(() -> MathUtil.applyDeadband(mPartner.getRightX(), 0.1), () -> MathUtil.applyDeadband(mPartner.getRightY(), 0.1), mCarriageSubsystem);
        mIntakeTester = new IntakeTester(mPartner::getAButton, mPartner::getBButton, mPartner::getXButton, mPartner::getYButton, mIntakeSubsystem);
        mClimberTester = new ClimberTester(mClimbSubsystem, mDriver::getLeftBumperButton, mDriver::getRightBumperButton, mDriver::getLeftTriggerAxis);
        // Configure subsystems
        mSwerveSubsystem.setDefaultCommand(mSwerveTeleop);
        // mElevatorCarriageSubsystem.setDefaultCommand(mElevatorCarriageTeleop);
        // mIntakeSubsystem.setDefaultCommand(mElevatorCarriageTeleop);

        // TODO: remove tester commands when robot is properly programmed
        mElevatorSubsystem.setDefaultCommand(mElevatorTester);
        mCarriageSubsystem.setDefaultCommand(mCarriageTester);
        mIntakeSubsystem.setDefaultCommand(mIntakeTester);
        mClimbSubsystem.setDefaultCommand(mClimberTester);

        // Configure other things.
        autoChooser = AutoBuilder.buildAutoChooser();

        // Register named commands.
        // TODO: Some of these are temporary things.
        NamedCommands.registerCommand("ScoreLevel3", new ScoreLevel3(mElevatorSubsystem, mCarriageSubsystem, mIntakeSubsystem));


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
