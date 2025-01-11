// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// This class is where subsystems and other robot parts are declared.
// IF A SUBSYSTEM IS NOT IN HERE, IT WILL NOT RUN!
public class RobotContainer
{
    // Ports go here:
    public static final int DriverControllerPort = 0;

    // Subsystems go here:
    private ExampleSubsystem mExampleSubsystem;

    // Controllers go here:
    // Replace with CommandPS4Controller or CommandJoystick if needed.
    private CommandXboxController mDriverController;

    public RobotContainer()
    {
        // Initialize subsystems.
        mExampleSubsystem = new ExampleSubsystem();

        configureSubsystems(); // Setup all the subsystems once they're created.
        
        // Initialize controllers.
        mDriverController = new CommandXboxController(DriverControllerPort);

        configureBindings(); // Setup controllers and triggers.
    }

    // If any first-time setup needs to be run on the subsystems, that code
    // would go here.
    private void configureSubsystems()
    {

    }

    // Configure conditions for events, such as controller buttons.
    private void configureBindings()
    {
        // When the example condition is met, it will start the example command
        // for the example subsystem.
        new Trigger(mExampleSubsystem::exampleCondition).onTrue(new ExampleCommand(mExampleSubsystem));

        // For as long as B is being pressed, run a method in the example
        // subsystem.
        mDriverController.b().whileTrue(mExampleSubsystem.exampleMethodCommand());
    }

    // Specify which command will be used as the autonomous command.
    public Command getAutonomousCommand()
    {
        // An example command will be run in autonomous
        return Autos.exampleAuto(mExampleSubsystem);
    }
}
