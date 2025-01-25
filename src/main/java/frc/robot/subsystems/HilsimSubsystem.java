package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HilsimSubsystem extends SubsystemBase
{
    public static final int kLightChannel = 7;

    private AddressableLED mLight;
    private AddressableLEDBuffer buffer;

    public HilsimSubsystem()
    {
        mLight = new AddressableLED(kLightChannel);

        buffer = new AddressableLEDBuffer(50);
        mLight.setLength(buffer.getLength());
        mLight.start();
    }

    int position = 0;
    int direction = 1;
    int length = 5;

    int shifterR = 255, shifterG = 0, shifterB = 0, state = 0;
    int pulserR = 0, pulserState = 1;

    @Override
    public void periodic()
    {
        AddressableLEDBufferView other = buffer.createView(25, 39);
        LEDPattern.rainbow(255, 128)
                  .scrollAtAbsoluteSpeed(MetersPerSecond.of(0.3), Meters.of(1e-2))
                  .applyTo(other);

        AddressableLEDBufferView left = null, green, right = null;

        if (position > 0) left = buffer.createView(0, position - 1);
        green = buffer.createView(position, position + length - 1);
        if (position + length - 1 < 24) right = buffer.createView(position + length, 24);

        LEDPattern off = LEDPattern.solid(Color.kBlack);
        LEDPattern greenColor = LEDPattern.solid(Color.kRed);

        if (left != null) off.applyTo(left);
        greenColor.applyTo(green);
        if (right != null) off.applyTo(right);

        position += direction;
        if (position + length - 1 == 23 || position == 0) direction *= -1;

        AddressableLEDBufferView colorShifter = buffer.createView(40, 44);
        LEDPattern shifted = LEDPattern.solid(new Color(shifterR, shifterG, shifterB));

        shifted.applyTo(colorShifter);

        switch (state)
        {
            case 0:
                shifterG++;
                if (shifterG == 255) state++;
                break;
            case 1:
                shifterR--;
                if (shifterR == 0) state++;
                break;
            case 2:
                shifterB++;
                if (shifterB == 255) state++;
                break;
            case 3:
                shifterG--;
                if (shifterG == 0) state++;
                break;
            case 4:
                shifterR++;
                if (shifterR == 255) state++;
                break;
            case 5:
                shifterB--;
                if (shifterB == 0) state = 0;
                break;
        }

        AddressableLEDBufferView colorPulser = buffer.createView(45, 49);
        pulserR += pulserState * 4;
        if (pulserR >= 255)
        {
            pulserR = 255;
            pulserState = -1;
        }
        else if (pulserR <= 0)
        {
            pulserR = 0;
            pulserState = 1;
        }
        LEDPattern pulserColor = LEDPattern.solid(new Color(pulserR, 0, 0));
        pulserColor.applyTo(colorPulser);

        mLight.setData(buffer);
    }

    @Override
    public void simulationPeriodic()
    {
        
    }
}
