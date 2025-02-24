package frc.robot.objectmodels.lightpatterns;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDReader;
import edu.wpi.first.wpilibj.LEDWriter;
import edu.wpi.first.wpilibj.util.Color;

public class LEDBouncingPattern implements LEDPattern
{
    private Color color;
    private int size;
    private int tick;

    public LEDBouncingPattern()
    {
        withDefaults();
    }

    public LEDBouncingPattern withColor(Color color)
    {
        this.color = color;
        return this;
    }
    public LEDBouncingPattern withSize(int size)
    {
        this.size = size;
        return this;
    }
    public LEDBouncingPattern withTick(int tick)
    {
        this.tick = tick;
        return this;
    }

    public LEDBouncingPattern withDefaults()
    {
        color = Color.kWhite;
        size = 5;
        tick = 0;
        return this;
    }

    public void applyTo(LEDReader reader, LEDWriter writer)
    {
        int length = reader.getLength();
        int tick = this.tick % ((length - size - 1) * 2);
        int startPos = 0, endPos = size;

        // This is a really brute-force method, but whatever. It works.
        int dir = 1;
        for (int i = 0; i < tick; i++)
        {
            startPos += dir;
            endPos += dir;
            if (endPos == length - 1) dir *= -1;
            else if (startPos == 0) dir *= -1;
        }

        for (int i = 0; i < length; i++)
        {
            Color col;
            if (i >= startPos && i <= endPos) col = color;
            else col = Color.kBlack;
            writer.setLED(i, col);
        }
    }
}
