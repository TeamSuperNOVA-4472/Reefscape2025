package frc.robot.objectmodels.lightpatterns;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDReader;
import edu.wpi.first.wpilibj.LEDWriter;
import edu.wpi.first.wpilibj.util.Color;

public class LEDSlidingPattern implements LEDPattern
{
    private Color color;
    private int onSize;
    private int offSize;
    private int tick;

    public LEDSlidingPattern()
    {
        withDefaults();
    }

    public LEDSlidingPattern withColor(Color color)
    {
        this.color = color;
        return this;
    }

    public LEDSlidingPattern withBothSize(int bothSize)
    {
        onSize = bothSize;
        offSize = bothSize;
        return this;
    }
    public LEDSlidingPattern withOffSize(int offSize)
    {
        this.offSize = offSize;
        return this;
    }
    public LEDSlidingPattern withOnSize(int onSize)
    {
        this.onSize = onSize;
        return this;
    }

    public LEDSlidingPattern withTick(int tick)
    {
        this.tick = tick;
        return this;
    }

    public LEDSlidingPattern withDefaults()
    {
        color = Color.kWhite;
        onSize = 5;
        offSize = 5;
        tick = 0;
        return this;
    }

    public void applyTo(LEDReader reader, LEDWriter writer)
    {
        int length = reader.getLength();
        int totalSize = onSize + offSize;
        int modTick = -tick % (onSize + offSize);
        if (modTick < 0) modTick += totalSize;

        for (int i = 0; i < length; i++)
        {
            int ref = (i + modTick) % totalSize;

            if (ref < onSize) writer.setLED(i, color);
            else writer.setLED(i, Color.kBlack);
        }
    }
}
