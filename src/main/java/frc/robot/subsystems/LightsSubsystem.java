package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.objectmodels.LightState;
import frc.robot.objectmodels.lightpatterns.RandomLEDPattern;

// Controls the light strips on the robot.
// I want the lights to be *very* well developed.
// I've created some custom LED patterns that I use here.
// They're meant to be used like builders.
// Ask me any questions you've got about this, I'd be happy to answer them!
public class LightsSubsystem extends SubsystemBase
{
    public static final int kLightChannel = 7;
    public static final int kLightCount = 25;

    private AddressableLED light;
    private AddressableLEDBuffer lightData;

    // Used in many animations. Reset between states.
    private int tick;

    // Oh yeah, it's a state machine with transitions. Gonna cry?
    // Sorry. I want transitions between multiple different animations,
    // and there isn't a much nicer way to do it.
    private LightState currentState;
    private Optional<LightState> newState;

    public LightsSubsystem()
    {
        light = new AddressableLED(kLightChannel);
        lightData = new AddressableLEDBuffer(kLightCount);
        light.setLength(kLightCount);
        light.start();

        currentState = LightState.kOff;
        newState = Optional.of(LightState.kDisconnected);
    }

    @Override
    public void periodic()
    {
        if (newState.isEmpty())
        {
            // Idle animations for each state.
            switch (currentState)
            {
                case kOff: animOff(); break;
                //case kDisconnected: animDisconnected(); break;

                default: animUnknown(); break;
            }
        }
        else
        {
            // Transitional animations from one state to another.
            // The method will complete the transition once it's finished.

            // TODO: Implement this. This is temporary code.
            currentState = newState.get();
            newState = Optional.empty();
        }

        light.setData(lightData);
        tick++;
    }

    // When the robot is off. Shouldn't ever really be used, this state
    // is mostly intended for transitioning out of when the robot wakes up.
    private void animOff()
    {
        LEDPattern off = LEDPattern.solid(Color.kBlack);
        off.applyTo(lightData);
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
