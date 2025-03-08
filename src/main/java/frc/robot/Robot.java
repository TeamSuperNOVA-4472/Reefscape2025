// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.objectmodels.LightState;
import frc.robot.objectmodels.LightStatusRequest;

// This code shouldn't be messed with in most scenarios.
// Most of this is run automatically.
public class Robot extends TimedRobot
{
    private static Robot self; // Kind of scuffed, but we need to expose instance methods as static ones.

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

        // Set up light status requests. Used for the base autonomous, teleop, and disabled effects.
        disabledLights = new LightStatusRequest(LightState.kDisabledStart, 1000);
        autonLights = new LightStatusRequest(LightState.kAutonomousBase, 100);
        teleopLights = new LightStatusRequest(LightState.kTeleopBase, 200);
        teleopLights.active = false;
        mRobotContainer.mLightsSubsystem.addRequests(disabledLights, autonLights, teleopLights);
        
        // Apply static variable.
        self = this;
    }

    public static boolean isRedAlliance()
    {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent())
        {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
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

    private LightStatusRequest disabledLights;
    // Run *once* when the robot becomes disabled.
    @Override
    public void disabledInit()
    {
        disabledLights.active = true;
    }
    // Run *once* when the robot is no longer disabled.
    @Override
    public void disabledExit()
    {
        disabledLights.active = false;
    }

    private LightStatusRequest autonLights;
    // Run *once* when the robot first goes into autonomous.
    @Override
    public void autonomousInit()
    {
        mAutonomousCommand = mRobotContainer.getAutonomousCommand();
        // TODO: Light state for when autonomous fails?
        autonLights.active = true;
        disabledLights.state = LightState.kDisabledError;

        // Schedule the autonomous command (example)
        if (mAutonomousCommand != null)
        {
            mAutonomousCommand.schedule();
        }
    }
    // Run *once* when the robot exits autonomous.
    @Override
    public void autonomousExit()
    {
        autonLights.active = false;
        disabledLights.state = LightState.kDisabledBetween;
    }

    private LightStatusRequest teleopLights;
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
        teleopLights.active = true;
        autonLights.active = false;
        disabledLights.state = LightState.kDisabledError;

        mRobotContainer.resetHeadingToAlliance();
    }

    // Run continuously when the robot is in teleop mode.
    @Override
    public void teleopPeriodic() { }

    @Override
    public void teleopExit()
    {
        teleopLights.active = false;
        disabledLights.state = LightState.kDisabledEnd;
    }

    public static boolean sIsTeleop()
    {
        return self.isTeleop();
    }
    public static boolean sIsAutonomous()
    {
        return self.isAutonomous();
    }

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
