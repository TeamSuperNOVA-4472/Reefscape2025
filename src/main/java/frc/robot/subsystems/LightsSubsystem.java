package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.objectmodels.LightState;
import frc.robot.objectmodels.LightStatusRequest;
import frc.robot.objectmodels.lightpatterns.LEDRandomFadeoutPattern;
import frc.robot.objectmodels.lightpatterns.LEDSweepingPattern;
import frc.robot.objectmodels.lightpatterns.RandomLEDPattern;

// Controls the light strips on the robot.
// I've created some custom LED patterns that I use here.
// Ask me any questions you've got about this, I'd be happy to answer them!
public class LightsSubsystem extends SubsystemBase
{
    public static final int kLightChannel = 7;
    public static final int kLightCount = 25;

    // This system uses the same buffer for each strip. This could be
    // changed if we want the two strips to show different stuff, but
    // I don't think we'll need or even want that.
    private AddressableLED light;
    private AddressableLEDBuffer lightData;

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
        SmartDashboard.putBoolean("Lights Active", isActive());
        if (!isActive()) return;
        LightState state = getActiveState();
        switch (state)
        {
            case kOff: animOff(); break;
            case kDisabledStart: animDisabled(Color.kPurple); break;
            case kAutonomous: animAuton(); break;
            case kDisabledBetween: animDisabled(Color.kBlue); break;
            case kDisabledError: animDisabled(Color.kOrange); break;
            case kDisabledEnd: animDisabled(Color.kGreen); break;
            default: animUnknown(); break;
        }
        SmartDashboard.putString("Light Strip State", state.toString());

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
        prevState = result;
        return result;
    }

    // When the robot is off. Shouldn't ever really be used, this state
    // is mostly intended for transitioning out of when the robot wakes up.
    private void animOff()
    {
        LEDPattern off = LEDPattern.solid(Color.kBlack);
        off.applyTo(lightData);
    }

    // When the robot is disabled.
    private void animDisabled(Color color)
    {
        new LEDSweepingPattern()
            .withColor(color)
            .moveMiddle()
            .tick((int)(0.25 * tick))
            .withSustain(0.9)
            .applyTo(lightData);
    }

    // When the robot is in autonomous. Matrix-style affect.
    private void animAuton()
    {
        new LEDRandomFadeoutPattern()
            .withColor(Color.kGreen)
            .withInterval(1)
            .withSustain(0.9)
            .tick(tick)
            .applyTo(lightData);
    }

    // When the robot is in an unknown state. Should never happen.
    private void animUnknown()
    {
        // TODO: Maybe replace this with a randomized fadeout? Could be cool.
        new RandomLEDPattern()
            .withSingleColor(Color.kRed)
            .withGamma(0.75)
            .applyTo(lightData);
    }
}
