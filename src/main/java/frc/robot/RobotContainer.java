// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DoTheThingCommand;
import frc.robot.commands.SwerveTeleop;
import frc.robot.subsystems.SwerveSubsystem;

import java.io.IOException;
import java.util.NoSuchElementException;
import java.util.Optional;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.AutoBuilderException;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {


  private final XboxController mDriver =
      new XboxController(OperatorConstants.kDriverControllerPort);

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem mSwerveSubsystem = new SwerveSubsystem();

  private final SlewRateLimiter mFwdLimiter = new SlewRateLimiter(1.0);
  private final SlewRateLimiter mSideLimiter = new SlewRateLimiter(1.0);
  private final SlewRateLimiter mTurnLimiter = new SlewRateLimiter(1.0);

  private final SendableChooser<Command> autoChooser;

  private final SwerveTeleop mSwerveTeleop = new SwerveTeleop(
    () -> mFwdLimiter.calculate(OperatorConstants.getControllerProfileValue(-mDriver.getLeftY())), 
    () -> mSideLimiter.calculate(OperatorConstants.getControllerProfileValue(-mDriver.getLeftX())),
    () -> mTurnLimiter.calculate(OperatorConstants.getControllerProfileValue(-mDriver.getRightX())),
    mDriver::getAButton,
    mSwerveSubsystem);



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser();
    mSwerveSubsystem.setDefaultCommand(mSwerveTeleop);

    NamedCommands.registerCommand("DoTheThingCommand", new DoTheThingCommand());
    NamedCommands.registerCommand("ScoreLevel4Right", new DoTheThingCommand());
    NamedCommands.registerCommand("ScoreLevel4Left", new DoTheThingCommand());
    NamedCommands.registerCommand("ScoreLevel3Right", new DoTheThingCommand());
    NamedCommands.registerCommand("IntakeCoral", new DoTheThingCommand());
    NamedCommands.registerCommand("GrabAlgae", new DoTheThingCommand());
    NamedCommands.registerCommand("ScoreAlgae", new DoTheThingCommand());

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


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
