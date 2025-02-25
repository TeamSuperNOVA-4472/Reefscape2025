package frc.robot.objectmodels.lightpatterns;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDReader;
import edu.wpi.first.wpilibj.LEDWriter;
import edu.wpi.first.wpilibj.util.Color;

public class LEDRandomFadeoutPattern implements LEDPattern
{
    private Color color;
    private int time;
    private double sustain;

    private int timer;

    public LEDRandomFadeoutPattern()
    {
        withDefaults();
    }

    /** Sets the base color for this pattern. */
    public LEDRandomFadeoutPattern withColor(Color color)
    {
        this.color = color;
        return this;
    }
    /** Waits this amount of ticks before setting a new LED. */
    public LEDRandomFadeoutPattern withInterval(int time)
    {
        this.time = time;
        return this;
    }
    /** The amount that each color gets multiplied by per tick. Scale of 0.0-1.0. */
    public LEDRandomFadeoutPattern withSustain(double sustain)
    {
        this.sustain = sustain;
        return this;
    }
    /** Sets the iteration or "tick" of the pattern. */
    public LEDRandomFadeoutPattern tick(int tick)
    {
        timer = tick;
        return this;
    }
    /** Resets this pattern's properties. */
    public LEDRandomFadeoutPattern withDefaults()
    {
        color = Color.kWhite;
        time = 1;
        sustain = 0.9;
        return this;
    }

    @Override
    public void applyTo(LEDReader reader, LEDWriter writer)
    {
        // Fade out and apply color to random LED.
        final int length = reader.getLength();

        // Fade out colors.
        for (int i = 0; i < length; i++)
        {
            Color oldColor = reader.getLED(i);
            Color newColor = new Color(
                oldColor.red * sustain,
                oldColor.green * sustain,
                oldColor.blue * sustain
            );
            writer.setLED(i, newColor);
        }

        if (timer % time == 0)
        {
            // Place new LED.
            int index = (int)(Math.random() * length);
            writer.setLED(index, color);
            timer -= time;
        }
    }
}
