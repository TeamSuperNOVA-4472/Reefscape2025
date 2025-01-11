// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

// This code shouldn't be messed with in most scenarios.
// Most of this is run automatically.
public class Robot extends TimedRobot
{
    private Command mAutonomousCommand;

    private final RobotContainer mRobotContainer;

    // Any global initialization code should go here.
    // This method is run when the robot first starts.
    // Do NOT put subsystem-specific code here.
    public Robot()
    {
        // Instantiate our RobotContainer. This will perform all our button
        // bindings, and put our autonomous chooser on the dashboard.
        mRobotContainer = new RobotContainer();
    }

    // This function is continuously called every 20 ms (50 times per second).
    // It always runs, even if the robot is disabled, in autonomous, or being
    // tested.
    @Override
    public void robotPeriodic()
    {
        // The scheduler polls button presses, handles commands, and runs
        // subsystem periodic methods. It really shouldn't be messed with.
        CommandScheduler.getInstance().run();
    }

    // Run *once* when the robot is first disabled.
    @Override
    public void disabledInit() { }

    // Run continuously when the robot is disabled.
    @Override
    public void disabledPeriodic() { }

    // Run *once* when the robot first goes into autonomous.
    @Override
    public void autonomousInit()
    {
        mAutonomousCommand = mRobotContainer.getAutonomousCommand();

        // Schedule the autonomous command (example)
        if (mAutonomousCommand != null)
        {
            mAutonomousCommand.schedule();
        }
    }

    // Run continuously when the robot is in autonomous mode.
    @Override
    public void autonomousPeriodic() { }

    // Run *once* when the robot first enters teleop mode.
    @Override
    public void teleopInit()
    {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (mAutonomousCommand != null)
        {
            mAutonomousCommand.cancel();
        }
    }

    // Run continuously when the robot is in teleop mode.
    @Override
    public void teleopPeriodic() { }

    // Run *once* when the robot is in test mode.
    // TODO: Is test mode the same thing as unit testing the robot?
    @Override
    public void testInit()
    {
        CommandScheduler.getInstance().cancelAll();
    }

    // Run continuously while the robot is in test mode.
    @Override
    public void testPeriodic() { }

    // Run *once* when the robot is being simulated.
    @Override
    public void simulationInit() { }

    // Runs continuously while the robot is being simulated.
    @Override
    public void simulationPeriodic() { }
}
