// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

// DO NOT MODIFY THIS FILE. Starts the robot.
// If you wish to make changes to this file, you are most likely trying to
// change something found in `RobotContainer`, or maybe `Robot`.
public final class Main
{
    private Main() { }

    public static void main(String... args)
    {
        // Starts the robot.
        RobotBase.startRobot(Robot::new);
    }
}
