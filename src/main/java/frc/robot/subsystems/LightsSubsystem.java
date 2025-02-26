package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.objectmodels.LightState;
import frc.robot.objectmodels.LightStatusRequest;
import frc.robot.objectmodels.lightpatterns.LEDBouncingPattern;
import frc.robot.objectmodels.lightpatterns.LEDRandomFadeoutPattern;
import frc.robot.objectmodels.lightpatterns.LEDSlidingPattern;
import frc.robot.objectmodels.lightpatterns.LEDSweepingPattern;
import frc.robot.objectmodels.lightpatterns.RandomLEDPattern;

// Controls the light strips on the robot.
// I've created some custom LED patterns that I use here.
// Ask me any questions you've got about this, I'd be happy to answer them!
public class LightsSubsystem extends SubsystemBase
{
    public static final int kLightChannel = 7;
    public static final int kLightCount = 50;

    // This system uses the same buffer for each strip. This could be
    // changed if we want the two strips to show different stuff, but
    // I don't think we'll need or even want that.
    private AddressableLED light;
    private AddressableLEDBuffer lightData;
    private AddressableLEDBufferView lightLeft, lightRight;

    // Used in many animations. Reset between states.
    private int tick;

    // We're using a priority system to prevent the need for
    // a state machine with conditions. The highest priority
    // status request that is active gets picked.
    // See `boolean getActiveState()`.
    private ArrayList<LightStatusRequest> requests;

    private LightState prevState;
    private boolean initialized, enabled = true;

    public LightsSubsystem()
    {
        lightData = new AddressableLEDBuffer(kLightCount);
        try
        {
            light = new AddressableLED(kLightChannel);
            light.setLength(kLightCount);
            light.start();
            initialized = true;
        }
        catch (Exception ex)
        {
            System.out.println("[LIGHTS] Failed to initialize the light strip. Lights will NOT be active.\n" +
                               "         Is the port to the light strip incorrect? It should be " + kLightChannel + ".");
            initialized = false;
            return;
        }

        // original, [0, 24] - [25, 49]
        lightLeft = lightData.createView(4, 20);
        lightRight = lightData.createView(29, 45);

        requests = new ArrayList<>();
        requests.add(new LightStatusRequest(LightState.kOff, 0));
    }

    public void addRequests(LightStatusRequest... requests)
    {
        if (!isActive()) return;
        for (int i = 0; i < requests.length; i++) addRequest(requests[i]);
    }
    public void addRequest(LightStatusRequest request)
    {
        if (!isActive()) return;
        for (int i = 0; i < requests.size(); i++)
        {
            LightStatusRequest req = requests.get(i);
            if (req.priority == request.priority &&
                req.state == request.state)
            {
                requests.set(i, request);
                return;
            }
        }
        requests.add(request);
    }

    public boolean isActive()
    {
        return initialized && enabled;
    }
    public void enable()
    {
        enabled = true;
    }
    public void disable()
    {
        enabled = false;
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putBoolean("Subsystems/LightsSubsystem/Light Status", isActive());
        if (!isActive()) return;
        LightState state = getActiveState();
        LEDPattern pattern;
        switch (state)
        {
            case kOff: pattern = animOff(); break;
            case kDisabledStart: pattern = animDisabled(Color.kPurple); break;
            case kAutonomousBase: pattern = animAuton(); break;
            case kAutonomousElevatorUp: pattern = animElevator(Color.kGreen, true); break;
            case kAutonomousElevatorDown: pattern = animElevator(Color.kGreen, false); break;
            case kDisabledBetween: pattern = animDisabled(Color.kBlue); break;
            case kTeleopBase: pattern = animTeleopBase(); break;
            case kTeleopElevatorUp: pattern = animElevator(Color.kRed, true); break;
            case kTeleopElevatorDown: pattern = animElevator(Color.kRed, false); break;
            case kDisabledError: pattern = animDisabled(Color.kOrange); break;
            case kDisabledEnd: pattern = animDisabled(Color.kGreen); break;
            default: pattern = animUnknown(); break;
        }

        pattern.applyTo(lightLeft);
        pattern.applyTo(lightRight);
        light.setData(lightData);
        tick++;
    }

    public LightState getActiveState()
    {
        if (!isActive()) return LightState.kOff;
        LightState result = LightState.kOff;
        int highest = Integer.MIN_VALUE;
        for (int i = 0; i < requests.size(); i++)
        {
            LightStatusRequest req = requests.get(i);
            if (req.active && req.priority >= highest)
            {
                result = req.state;
                highest = req.priority;
            }
        }
        if (result != prevState)
        {
            System.out.println("[LIGHTS] Switching state: " + prevState + " -> " + result + ".");
        }
        SmartDashboard.putString("Subsystems/LightsSubsystem/Light State", result.toString());
        prevState = result;
        return result;
    }

    // When the robot is off. Shouldn't ever really be used, this state
    // is mostly intended for transitioning out of when the robot wakes up.
    private LEDPattern animOff()
    {
        return LEDPattern.kOff;
    }

    // When the robot is disabled. Different colors for different states.
    private LEDPattern animDisabled(Color color)
    {
        return new LEDSweepingPattern()
            .withColor(color)
            .moveMiddle()
            .tick((int)(0.25 * tick))
            .withSustain(0.9);
    }

    // When the robot is in autonomous. Matrix-style affect.
    private LEDPattern animAuton()
    {
        return new LEDRandomFadeoutPattern()
            .withColor(Color.kGreen)
            .withInterval(1)
            .withSustain(0.9)
            .tick(tick);
    }

    // When the robot is in driver control. Going for a supernova-colored bouncing effect.
    private LEDPattern animTeleopBase()
    {
        return new LEDBouncingPattern()
            .withColor(Color.kRed)
            .withSize(5)
            .withTick((int)(tick / 3.0 * 2));
    }

    // Play a cool sliding effect when the elevator is moving 
    private LEDPattern animElevator(Color color, boolean up)
    {
        return new LEDSlidingPattern()
            .withColor(color)
            .withOnSize(5)
            .withOffSize(10)
            .withTick((tick / 2) * (up ? 1 : -1));
    }

    // When the robot is in an unknown state. Should never happen.
    private LEDPattern animUnknown()
    {
        // TODO: Maybe replace this with a randomized fadeout? Could be cool.
        return new RandomLEDPattern()
            .withSingleColor(Color.kRed)
            .withGamma(0.75);
    }
}
