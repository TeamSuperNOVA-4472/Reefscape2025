// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public final class Constants
{
    public static final String kCanivoreBusName = "CANivore";
    public static final double kMaxVolts = 12.0;
    public static class OperatorConstants
    {
        public static final double kDeadband = 0.1;

        // This is meant to be a weighted system, correct?
        // TODO: A better description is probably in order.
        public static final InterpolatingDoubleTreeMap kControllerProfileMap = new InterpolatingDoubleTreeMap();

        static
        {
            kControllerProfileMap.put(0.0, 0.0);
            kControllerProfileMap.put(1.0, 1.0);
        }

        // TODO: I don't think this belongs here.
        //       I'd rather not have a constants file. Maybe some of these could be constants/methods in RobotContainer?
        //       We could also have a file specifically for operator stuff.
        public static double getControllerProfileValue(double pValue)
        {
            double deadbandedVal = MathUtil.applyDeadband(pValue, kDeadband);
            double clampedVal = MathUtil.clamp(deadbandedVal, -1, 1);
            return Math.signum(clampedVal) * kControllerProfileMap.get(Math.abs(clampedVal));
        }
    }
}
