package frc.robot.objectmodels.lightpatterns;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDReader;
import edu.wpi.first.wpilibj.LEDWriter;
import edu.wpi.first.wpilibj.util.Color;

public class LEDSweepingPattern implements LEDPattern
{
    private Color color;
    private Direction direction;
    private int tick;
    private double sustain;

    public LEDSweepingPattern()
    {
        withDefaults();
    }

    /** Sets the color for this pattern. */
    public LEDSweepingPattern withColor(Color color)
    {
        this.color = color;
        return this;
    }
    /** Sets the direction for this pattern. */
    public LEDSweepingPattern withDirection(Direction direction)
    {
        this.direction = direction;
        return this;
    }
    /** Makes the pattern travel to the left. */
    public LEDSweepingPattern moveLeft()
    {
        direction = Direction.kLeft;
        return this;
    }
    /** Makes the pattern travel to the right. */
    public LEDSweepingPattern moveRight()
    {
        direction = Direction.kRight;
        return this;
    }
    /** Makes the pattern start in the middle and move in both directions. */
    public LEDSweepingPattern moveMiddle()
    {
        direction = Direction.kMiddle;
        return this;
    }
    /** Sets the iteration or "tick" of the pattern. */
    public LEDSweepingPattern tick(int tick)
    {
        this.tick = tick;
        return this;
    }
    /** The amount that each color gets multiplied by per tick. Scale of 0.0-1.0. */
    public LEDSweepingPattern withSustain(double sustain)
    {
        this.sustain = sustain;
        return this;
    }
    /** Resets this pattern's properties. */
    public LEDSweepingPattern withDefaults()
    {
        color = Color.kWhite;
        direction = Direction.kRight;
        tick = 0;
        sustain = 0.9;
        return this;
    }

    @Override
    public void applyTo(LEDReader reader, LEDWriter writer)
    {
        // Fade out and apply color if it's at the right spot.
        int length = reader.getLength();

        int[] toSet;
        switch (direction)
        {
            case kLeft:
                toSet = new int[] { tick % length };
                break;

            case kRight:
                toSet = new int[] { length - (tick % length) - 1 };
                break;

            case kMiddle:
                if (length % 2 == 0)
                {
                    // Even length.
                    int middle = length / 2;
                    int offset = tick % middle;
                    toSet = new int[] { middle - offset - 1, middle + offset };
                }
                else
                {
                    // Odd length.
                    int offset = tick % (length / 2 + 1);
                    toSet = new int[] { ((length + 1) / 2) - offset - 1, ((length - 1) / 2) + offset };
                }
                break;

            default:
                toSet = new int[0];
                break;
        }

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
        
        // Fully refresh new colors.
        for (int j = 0; j < toSet.length; j++)
        {
            // toSet contains the indices of the LEDs to refresh.
            int i = toSet[j];
            writer.setLED(i, color);
        }
    }

    public enum Direction
    {
        kLeft,
        kMiddle,
        kRight
    }
}
