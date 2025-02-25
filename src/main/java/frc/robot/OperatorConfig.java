package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

// Handles driver configuration, such as weighted joystick inputs and deadband.
public final class OperatorConfig
{
    public static final double kJoystickDeadband = 0.1;

    // Represents the weighted joystick function.
    public static final InterpolatingDoubleTreeMap kControllerProfileMap = new InterpolatingDoubleTreeMap();

    static
    {
        // This function takes in a linear joystick value and weights it.
        // In the real world, we probably care about the difference between
        // 5% speed and 10% speed more than we care about the difference
        // between 90% speed and 95% speed, so we associate more of the
        // joystick with smaller values.

        // Here, the left argument represents the X-axis, and the right
        // argument represents the Y-axis on a graph. For instance, the first
        // 80% of the joystick goes between 0% and 30% actual robot velocity,
        // and the last 20% is what goes between 30% and 100%. It looks like of
        // like this:
        // 
        // |                       :
        // |                      :
        // |                     :
        // |                     :
        // |                    :
        // |                   .:
        // |            .......:
        // |      .....:
        // |.....:
        // └------------------------

        kControllerProfileMap.put(0.0, 0.0);
        //kControllerProfileMap.put(0.8, 0.3);
        kControllerProfileMap.put(1.0, 1.0);
    }

    /** Apply deadband and weight to a linear joystick input. */
    public static final double weightedJoystickValue(double joystickIn)
    {
        double deadbandVal = MathUtil.applyDeadband(joystickIn, kJoystickDeadband);
        double clamped = MathUtil.clamp(deadbandVal, -1.0, 1.0);
        double magnitude = Math.abs(clamped);
        return Math.signum(clamped) * kControllerProfileMap.get(magnitude);
    }

    /** Weight a supplier representing joystick inputs. */
    public static final Supplier<Double> weightJoystick(Supplier<Double> joystick, boolean invert)
    {
        return () ->
        {
            double val = weightedJoystickValue(joystick.get());
            return invert ? -val : val;
        };
    }
    /** Weight a supplier representing joystick inputs. */
    public static final Supplier<Double> weightJoystick(Supplier<Double> joystick)
    {
        return weightJoystick(joystick, false);
    }
}
