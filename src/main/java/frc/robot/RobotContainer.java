// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.OperatorConfig.weightJoystick;
import frc.robot.commands.DriveDistanceAndHeading;
import frc.robot.commands.ElevatorCarriageTeleop;
import frc.robot.commands.PresetTeleop;
import frc.robot.commands.SwerveTeleop;
import frc.robot.commands.SwitchPresetCommand;
import frc.robot.commands.VisionAlignCommand;
import frc.robot.commands.tester.CarriageTester;
import frc.robot.commands.tester.ClimberTester;
import frc.robot.commands.tester.ElevatorTester;
import frc.robot.commands.tester.IntakeTester;
import frc.robot.objectmodels.CarriagePreset;
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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.OperatorConfig.weightJoystick;
import frc.robot.subsystems.VisionSubsystem;

import java.util.Optional;

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
    //private final CarriageSubsystem mCarriageSubsystem;
    private final ElevatorCarriageSubsystem mElevatorCarriageSubsystem;
    //private final ElevatorSubsystem mElevatorSubsystem;
    private final IntakeSubsystem mIntakeSubsystem;
    private final SwerveSubsystem mSwerveSubsystem;
    // private final ClimbSubsystem mClimbSubsystem;

    // Controllers go here:
    private final XboxController mDriver;
    private final XboxController mPartner;
    
    // Commands go here:
    private final SwerveTeleop mSwerveTeleop;
    //private final ElevatorCarriageTeleop mElevatorCarriageTeleop;
    //private final IntakeTeleop mIntakeTeleop;

    // TODO: Remove tester commands when robot is properly programmed
    private final IntakeTester mIntakeTester;
    // private final ClimberTester mClimberTester;

    // TODO: In the future I think more triggers should be brought out here.
    // private final Trigger algaeTrigger, algaeHighTrigger;

    // Extras:
    private final SendableChooser<Command> autoChooser;

    public RobotContainer()
    {
        // Initialize controllers.
        mDriver = new XboxController(kDriverPort);
        mPartner = new XboxController(kPartnerPort);

        // Initialize subsystems.
        mLightsSubsystem = LightsSubsystem.kInstance;
        mSwerveSubsystem = SwerveSubsystem.kInstance;
        mVisionSubsystem = VisionSubsystem.kInstance;
        mElevatorCarriageSubsystem = ElevatorCarriageSubsystem.kInstance;
        mIntakeSubsystem = IntakeSubsystem.kInstance;

        // Initialize commands.
        // TODO: Should weighting go here? Or in the command?
        //       Also, we should add a comment explaining why
        //       we need to invert these.
        mSwerveTeleop = new SwerveTeleop(
            weightJoystick(mDriver::getLeftY, true),
            weightJoystick(mDriver::getLeftX, true),
            weightJoystick(mDriver::getRightX, true),
            mDriver::getAButton);

        // TODO: remove tester commands when robot is properly programmed
        //mElevatorTester = new ElevatorTester(mElevatorSubsystem, () -> MathUtil.applyDeadband(-mPartner.getLeftY(), 0.1));
        //mCarriageTester = new CarriageTester(() -> MathUtil.applyDeadband(mPartner.getRightX(), 0.1), () -> MathUtil.applyDeadband(mPartner.getRightY(), 0.1), mCarriageSubsystem);
        mIntakeTester = new IntakeTester(mPartner::getAButton, mPartner::getBButton, mPartner::getXButton, mPartner::getYButton, mIntakeSubsystem);
        // mClimberTester = new ClimberTester(mClimbSubsystem, mDriver::getLeftBumperButton, mDriver::getRightBumperButton, mDriver::getLeftTriggerAxis);
        // Configure subsystems
        // Kyle here. Sophia wants her controls to be disabled when moving the arms in.
        // This is the fastest fix I could make.
        mSwerveSubsystem.setDefaultCommand(mSwerveTeleop);
        /*mVisionSubsystem.addMeasurementListener((EstimatedRobotPose newVisionPose) -> {
            // Update the swerve's odometry with the new vision estimate.
            mSwerveSubsystem.addVisionMeasurement(newVisionPose.estimatedPose.toPose2d(),
                                                  newVisionPose.timestampSeconds);
        });*/ // FIXME: This causes weird things.

        

        // TODO: remove tester commands when robot is properly programmed
        //mElevatorSubsystem.setDefaultCommand(mElevatorTester);
        //mCarriageSubsystem.setDefaultCommand(mCarriageTester);
        mIntakeSubsystem.setDefaultCommand(mIntakeTester);
        PresetTeleop.setup(mPartner);

        // mClimbSubsystem.setDefaultCommand(mClimberTester);

        // Register named commands.
        NamedCommands.registerCommand("StowCarriage", SwitchPresetCommand.stow(false));
        NamedCommands.registerCommand("MoveCoralL1", new SwitchPresetCommand(CarriagePreset.kCoralL1, false));
        NamedCommands.registerCommand("MoveCoralL2", new SwitchPresetCommand(CarriagePreset.kCoralL2, false));
        NamedCommands.registerCommand("MoveCoralL3", new SwitchPresetCommand(CarriagePreset.kCoralL3, false));
        NamedCommands.registerCommand("MoveCoralL4", new SwitchPresetCommand(CarriagePreset.kCoralL4, false));
        NamedCommands.registerCommand("ReefVisionAlignLeft", new VisionAlignCommand(VisionAlignCommand.kReefLeftOffset, Optional.empty()));
        NamedCommands.registerCommand("ReefVisionAlignMiddle", new VisionAlignCommand(VisionAlignCommand.kReefMiddleOffset, Optional.empty()));
        NamedCommands.registerCommand("ReefVisionAlignRight", new VisionAlignCommand(VisionAlignCommand.kReefRightOffset, Optional.empty()));
        NamedCommands.registerCommand("MoveAlgaeL2", new SwitchPresetCommand(CarriagePreset.kAlgaeL2, false));
        NamedCommands.registerCommand("MoveAlgaeL3", new SwitchPresetCommand(CarriagePreset.kAlgaeL3, false));
        NamedCommands.registerCommand("IntakeCoral", new InstantCommand(() -> mIntakeSubsystem.intakeCoral()));
        NamedCommands.registerCommand("OuttakeCoral", new InstantCommand(() -> mIntakeSubsystem.outtakeCoral()));
        NamedCommands.registerCommand("IntakeAlgae", new InstantCommand(() -> mIntakeSubsystem.intakeAlgae()));
        NamedCommands.registerCommand("OuttakeAlgae", new InstantCommand(() -> mIntakeSubsystem.outtakeAlgae()));
        NamedCommands.registerCommand("StopIntake", new InstantCommand(() -> mIntakeSubsystem.stopBoth()));

        // Configure other things.
        autoChooser = AutoBuilder.buildAutoChooser();

        // TODO: DEBUG THING, PLEASE REMOVE
        new EventTrigger("TheEvent").onTrue(
                new InstantCommand(() -> System.out.println("The Event has triggered")));

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    // private void applyHeightOffsetWhenVisionAlignFinishes()
    // {
    //     // Scuffed as hell. As a completely last step, apply Christian's
    //     // offsets for the different levels.

    //     Translation2d offset;
    //     if (mPartner.getYButton()) offset = VisionAlignCommand.kDeltaForL4;
    //     else if (mPartner.getBButton()) offset = VisionAlignCommand.kDeltaForL3;
    //     else if (mPartner.getXButton()) offset = VisionAlignCommand.kDeltaForL2;
    //     else if (mPartner.getAButton()) offset = VisionAlignCommand.kDeltaForL1;
    //     else offset = Translation2d.kZero;

    //     // If in algae preset, go a little more towards the reef.
    //     if (algaeTrigger.getAsBoolean() || algaeHighTrigger.getAsBoolean())
    //         offset = offset.plus(VisionAlignCommand.kDeltaForAlgae);

    //     Pose2d pose = new Pose2d(offset, Rotation2d.kZero);
    //     DriveDistanceAndHeading moveCmd = new DriveDistanceAndHeading(() -> pose);
    //     moveCmd.schedule(); // I really hope this works.
    // }

    // Specify which command will be used as the autonomous command.
    public Command getAutonomousCommand()
    {
        return autoChooser.getSelected();
    }

    /**
     * Used to establish field oriented control to the correct alliance side
     */
    public void resetHeadingToAlliance() {
        mSwerveTeleop.resetHeadingToAlliance();
    }

}
