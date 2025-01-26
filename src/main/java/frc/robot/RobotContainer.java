// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DoTheThingCommand;
import frc.robot.commands.DriveDistanceAndHeading;
import frc.robot.commands.SwerveTeleop;
import frc.robot.subsystems.SwerveSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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

  // Extras:
  private final SendableChooser<Command> autoChooser;

  private final SwerveTeleop mSwerveTeleop = new SwerveTeleop(
    () -> mFwdLimiter.calculate(OperatorConstants.getControllerProfileValue(-mDriver.getLeftY())), 
    () -> mSideLimiter.calculate(OperatorConstants.getControllerProfileValue(-mDriver.getLeftX())),
    () -> mTurnLimiter.calculate(OperatorConstants.getControllerProfileValue(-mDriver.getRightX())),
    mDriver::getAButton,
    mSwerveSubsystem);



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    mSwerveSubsystem.setDefaultCommand(mSwerveTeleop);
    NamedCommands.registerCommand("DoTheThingCommand", new DoTheThingCommand());
    new EventTrigger("TheEvent").onTrue(
      new InstantCommand(() -> System.out.println("The Event has triggered")));
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

      SmartDashboard.putData("Auto Chooser", autoChooser);
      Trigger xTrigger = new Trigger(mDriver::getXButton);
      xTrigger.whileTrue(new DriveDistanceAndHeading(
        mSwerveSubsystem,
        new Translation2d(0.5, 0.5),
        90));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //return autoChooser.getSelected();
    return new DriveDistanceAndHeading(
      mSwerveSubsystem,
      new Translation2d(0.5,0.5),
      90);
  }
}
