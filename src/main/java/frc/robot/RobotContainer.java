// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;
import java.util.Set;

import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static frc.robot.OperatorConfig.weightJoystick;
import frc.robot.commands.PresetTeleop;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ClimbTeleop;
import frc.robot.commands.DoTheThingCommand;
import frc.robot.commands.SwerveTeleop;
import frc.robot.commands.SwitchPresetCommand;
import frc.robot.commands.VisionAlign;
import frc.robot.commands.VisionAlignCommand;
import frc.robot.commands.VisionTeleop;
import frc.robot.commands.tester.CarriageTester;
import frc.robot.commands.tester.ElevatorTester;
import frc.robot.commands.tester.IntakeTester;
import frc.robot.objectmodels.CarriagePreset;
import frc.robot.objectmodels.ReefEndTarget;
import frc.robot.objectmodels.VisionDirection;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevatorCarriageSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// This class is where subsystems and other robot parts are declared.
// IF A SUBSYSTEM IS NOT IN HERE, IT WILL NOT RUN!
@SuppressWarnings("unused")
public class RobotContainer {
    // Ports go here:
    public static final int kDriverPort = 0;
    public static final int kPartnerPort = 1;

    // Subsystems go here:
    public LightsSubsystem mLightsSubsystem;
    private VisionSubsystem mVisionSubsystem;
    private final ElevatorCarriageSubsystem mElevatorCarriageSubsystem;
    private final IntakeSubsystem mIntakeSubsystem;
    private final SwerveSubsystem mSwerveSubsystem;
    private final ClimbSubsystem mClimbSubsystem;

    // Controllers go here:
    private final XboxController mDriver;
    private final XboxController mPartner;

    // Commands go here:
    private final SwerveTeleop mSwerveTeleop;
    private final ClimbTeleop mClimbTeleop;

    // TODO: Remove tester commands when robot is properly programmed
    private final IntakeTester mIntakeTester;

    // Extras:
    private final SendableChooser<Command> autoChooser;
    private boolean isFirst;

    public RobotContainer() {
        // Initialize controllers.
        mDriver = new XboxController(kDriverPort);
        mPartner = new XboxController(kPartnerPort);

        // Initialize subsystems.
        mClimbSubsystem = ClimbSubsystem.kInstance;
        mLightsSubsystem = LightsSubsystem.kInstance;
        mSwerveSubsystem = SwerveSubsystem.kInstance;
        mVisionSubsystem = VisionSubsystem.kInstance;
        mElevatorCarriageSubsystem = ElevatorCarriageSubsystem.kInstance;
        mIntakeSubsystem = IntakeSubsystem.kInstance;

        isFirst = true;

        // Initialize commands.
        // TODO: Should weighting go here? Or in the command?
        // Also, we should add a comment explaining why
        // we need to invert these.
        mSwerveTeleop = new SwerveTeleop(
                weightJoystick(mDriver::getLeftY, true),
                weightJoystick(mDriver::getLeftX, true),
                weightJoystick(mDriver::getRightX, true),
                mDriver::getAButton, () -> mDriver.getBButton());

        // TODO: remove tester commands when robot is properly programmed
        mIntakeTester = new IntakeTester(mPartner::getAButton, mPartner::getBButton, mPartner::getXButton,
                mPartner::getYButton, mIntakeSubsystem);
        mClimbTeleop = new ClimbTeleop(mDriver::getLeftBumperButtonPressed, mDriver::getRightBumperButtonPressed);
        // Configure subsystems
        // Kyle here. Sophia wants her controls to be disabled when moving the arms in.
        // This is the fastest fix I could make.
        mSwerveSubsystem.setDefaultCommand(mSwerveTeleop);
        mClimbSubsystem.setDefaultCommand(mClimbTeleop);
        mIntakeSubsystem.setDefaultCommand(mIntakeTester);
        PresetTeleop.setup(mPartner);
        VisionTeleop.setup(mDriver);

        mVisionSubsystem.addMeasurementListener((EstimatedRobotPose newVisionPose) -> {
            // Update the swerve's odometry with the new vision estimate.
            /*
             * Pose2d vision = newVisionPose.estimatedPose.toPose2d();
             * Pose2d current = mSwerveSubsystem.getPose();
             * 
             * Pose2d measurement = new Pose2d(vision.getTranslation(),
             * current.getRotation());
             */
            mSwerveSubsystem.addVisionMeasurement(newVisionPose.estimatedPose.toPose2d(),
                    newVisionPose.timestampSeconds);
            isFirst = false;
        });

        VisionAlign visionAlign = new VisionAlign(mSwerveSubsystem, mVisionSubsystem);

        // Register named commands.
        NamedCommands.registerCommand("StowCarriage", SwitchPresetCommand.stow(false));
        NamedCommands.registerCommand("MoveCoralL1", new SwitchPresetCommand(CarriagePreset.kCoralL1, false));
        NamedCommands.registerCommand("MoveCoralL2", new SwitchPresetCommand(CarriagePreset.kCoralL2, false));
        NamedCommands.registerCommand("MoveCoralL3", new SwitchPresetCommand(CarriagePreset.kCoralL3, false));
        NamedCommands.registerCommand("MoveCoralL4", new SwitchPresetCommand(CarriagePreset.kCoralL4, false));
        NamedCommands.registerCommand("MoveAlgaeL2", new SwitchPresetCommand(CarriagePreset.kAlgaeL2, false));
        NamedCommands.registerCommand("MoveAlgaeL3", new SwitchPresetCommand(CarriagePreset.kAlgaeL3, false));
        NamedCommands.registerCommand("IntakeCoral", new InstantCommand(() -> mIntakeSubsystem.intakeCoral()));
        NamedCommands.registerCommand("OuttakeCoral", new InstantCommand(() -> mIntakeSubsystem.outtakeCoral()));
        NamedCommands.registerCommand("IntakeAlgae", new InstantCommand(() -> mIntakeSubsystem.intakeAlgae()));
        NamedCommands.registerCommand("OuttakeAlgae", new InstantCommand(() -> mIntakeSubsystem.outtakeAlgae()));
        NamedCommands.registerCommand("StopIntake", new InstantCommand(() -> mIntakeSubsystem.stopBoth()));
        NamedCommands.registerCommand("Loading", SwitchPresetCommand.load(false));

        // Align to Reef named commands. There is going to be a lot
        NamedCommands.registerCommand("AlignToFarBottom-Right", new DeferredCommand(
                () -> new VisionAlign(
                        mSwerveSubsystem, mVisionSubsystem).alignToReef(ReefEndTarget.FarRight,
                                VisionDirection.RightCoral, CarriagePreset.kCoralL4, false),
                Set.of(mSwerveSubsystem, mVisionSubsystem)));

        NamedCommands.registerCommand("AlignToFarBottom-Left", new DeferredCommand(
                () -> new VisionAlign(
                        mSwerveSubsystem, mVisionSubsystem).alignToReef(ReefEndTarget.FarRight,
                                VisionDirection.LeftCoral, CarriagePreset.kCoralL4, false),
                Set.of(mSwerveSubsystem, mVisionSubsystem)));

        NamedCommands.registerCommand("AlignToFarRight-Algae", new DeferredCommand(
                () -> new VisionAlign(
                        mSwerveSubsystem, mVisionSubsystem).alignToReef(ReefEndTarget.FarRight,
                                VisionDirection.MiddleDefault, CarriagePreset.kCoralL4, false),
                Set.of(mSwerveSubsystem, mVisionSubsystem)));

        NamedCommands.registerCommand("AlignToFarLeft-Right", new DeferredCommand(
                () -> new VisionAlign(
                        mSwerveSubsystem, mVisionSubsystem).alignToReef(ReefEndTarget.FarLeft,
                                VisionDirection.RightCoral, CarriagePreset.kCoralL4, false),
                Set.of(mSwerveSubsystem, mVisionSubsystem)));

        NamedCommands.registerCommand("AlignToFarLeft-Left", new DeferredCommand(
                () -> new VisionAlign(
                        mSwerveSubsystem, mVisionSubsystem).alignToReef(ReefEndTarget.FarLeft,
                                VisionDirection.LeftCoral, CarriagePreset.kCoralL4, false),
                Set.of(mSwerveSubsystem, mVisionSubsystem)));

        NamedCommands.registerCommand("AlignToFarLeft-Algae", new DeferredCommand(
                () -> new VisionAlign(
                        mSwerveSubsystem, mVisionSubsystem).alignToReef(ReefEndTarget.FarLeft,
                                VisionDirection.MiddleDefault, CarriagePreset.kCoralL4, false),
                Set.of(mSwerveSubsystem, mVisionSubsystem)));

        NamedCommands.registerCommand("AlignToFarMiddle-Right", new DeferredCommand(
                () -> new VisionAlign(
                        mSwerveSubsystem, mVisionSubsystem).alignToReef(ReefEndTarget.FarMiddle,
                                VisionDirection.RightCoral, CarriagePreset.kCoralL4, false),
                Set.of(mSwerveSubsystem, mVisionSubsystem)));

        NamedCommands.registerCommand("AlignToFarMiddle-Left", new DeferredCommand(
                () -> new VisionAlign(
                        mSwerveSubsystem, mVisionSubsystem).alignToReef(ReefEndTarget.FarMiddle,
                                VisionDirection.LeftCoral, CarriagePreset.kCoralL4, false),
                Set.of(mSwerveSubsystem, mVisionSubsystem)));

        NamedCommands.registerCommand("AlignToFarMiddle-Algae", new DeferredCommand(
                () -> new VisionAlign(
                        mSwerveSubsystem, mVisionSubsystem).alignToReef(ReefEndTarget.FarMiddle,
                                VisionDirection.MiddleDefault, CarriagePreset.kCoralL4, false),
                Set.of(mSwerveSubsystem, mVisionSubsystem)));

        NamedCommands.registerCommand("AlignToNearMiddle-Right", new DeferredCommand(
                () -> new VisionAlign(
                        mSwerveSubsystem, mVisionSubsystem).alignToReef(ReefEndTarget.NearMiddle,
                                VisionDirection.RightCoral, CarriagePreset.kCoralL4, false),
                Set.of(mSwerveSubsystem, mVisionSubsystem)));

        NamedCommands.registerCommand("AlignToNearMiddle-Left", new DeferredCommand(
                () -> new VisionAlign(
                        mSwerveSubsystem, mVisionSubsystem).alignToReef(ReefEndTarget.NearMiddle,
                                VisionDirection.LeftCoral, CarriagePreset.kCoralL4, false),
                Set.of(mSwerveSubsystem, mVisionSubsystem)));

        NamedCommands.registerCommand("AlignToNearMiddle-Algae", new DeferredCommand(
                () -> new VisionAlign(
                        mSwerveSubsystem, mVisionSubsystem).alignToReef(ReefEndTarget.NearMiddle,
                                VisionDirection.MiddleDefault, CarriagePreset.kCoralL4, false),
                Set.of(mSwerveSubsystem, mVisionSubsystem)));

        NamedCommands.registerCommand("AlignToNearRight-Right", new DeferredCommand(
                () -> new VisionAlign(
                        mSwerveSubsystem, mVisionSubsystem).alignToReef(ReefEndTarget.NearRight,
                                VisionDirection.RightCoral, CarriagePreset.kCoralL4, false),
                Set.of(mSwerveSubsystem, mVisionSubsystem)));

        NamedCommands.registerCommand("AlignToNearRight-Left", new DeferredCommand(
                () -> new VisionAlign(
                        mSwerveSubsystem, mVisionSubsystem).alignToReef(ReefEndTarget.NearRight,
                                VisionDirection.LeftCoral, CarriagePreset.kCoralL4, false),
                Set.of(mSwerveSubsystem, mVisionSubsystem)));

        NamedCommands.registerCommand("AlignToNearRight-Algae", new DeferredCommand(
                () -> new VisionAlign(
                        mSwerveSubsystem, mVisionSubsystem).alignToReef(ReefEndTarget.NearRight,
                                VisionDirection.MiddleDefault, CarriagePreset.kCoralL4, false),
                Set.of(mSwerveSubsystem, mVisionSubsystem)));

        NamedCommands.registerCommand("AlignToNearLeft-Right", new DeferredCommand(
                () -> new VisionAlign(
                        mSwerveSubsystem, mVisionSubsystem).alignToReef(ReefEndTarget.NearLeft,
                                VisionDirection.RightCoral, CarriagePreset.kCoralL4, false),
                Set.of(mSwerveSubsystem, mVisionSubsystem)));

        NamedCommands.registerCommand("AlignToNearLeft-Left", new DeferredCommand(
                () -> new VisionAlign(
                        mSwerveSubsystem, mVisionSubsystem).alignToReef(ReefEndTarget.NearLeft,
                                VisionDirection.LeftCoral, CarriagePreset.kCoralL4, false),
                Set.of(mSwerveSubsystem, mVisionSubsystem)));

        NamedCommands.registerCommand("AlignToNearLeft-Algae", new DeferredCommand(
                () -> new VisionAlign(
                        mSwerveSubsystem, mVisionSubsystem).alignToReef(ReefEndTarget.NearLeft,
                                VisionDirection.MiddleDefault, CarriagePreset.kCoralL4, false),
                Set.of(mSwerveSubsystem, mVisionSubsystem)));

        NamedCommands.registerCommand("AlignToBottomLoading", new DeferredCommand(
                () -> new VisionAlign(
                        mSwerveSubsystem, mVisionSubsystem).alignToRightMatchLoadingStation(),
                Set.of(mSwerveSubsystem, mVisionSubsystem)));

        NamedCommands.registerCommand("AlignToLeftLoading", new DeferredCommand(
                () -> new VisionAlign(
                        mSwerveSubsystem, mVisionSubsystem).alignToLeftMatchLoadingStation(),
                Set.of(mSwerveSubsystem, mVisionSubsystem)));

        // Configure other things.
        autoChooser = new SendableChooser<>();//AutoBuilder.buildAutoChooser();
        autoChooser.addOption("ReefFarLeft2L4", new PathPlannerAuto("ReefFarLeft2L4"));
        autoChooser.addOption("ReefFarRight2L4", new PathPlannerAuto("ReefFarRight2L4"));
        autoChooser.setDefaultOption("FarMiddleL4-Right", new PathPlannerAuto("FarMiddleL4-Right"));
        SmartDashboard.putData("Auto Selector", autoChooser);
    }

    // Specify which command will be used as the autonomous command.
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    /**
     * Used to establish field oriented control to the correct alliance side
     */
    public void resetHeadingToAlliance() {
        mSwerveTeleop.resetHeadingToAlliance();
    }

}
