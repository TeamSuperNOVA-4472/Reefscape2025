package frc.robot.objectmodels.lightpatterns;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDReader;
import edu.wpi.first.wpilibj.LEDWriter;
import edu.wpi.first.wpilibj.util.Color;

/** Makes every LED a random color. Has the capacity to
    stick to a gradient, single shade, or mono. */
// I GIVE YOU OVERENGINEERED RANDOM LED PATTERN
public class RandomLEDPattern implements LEDPattern
{
    private boolean isGradient;
    private Color colorA, colorB;
    private double gamma = 1.0;

    public RandomLEDPattern()
    {
        isGradient = false;
        colorA = null;
        colorB = null;
    }

    /** Chooses a random color between black and white. */
    public RandomLEDPattern withMono()
    {
        return withGradient(Color.kBlack, Color.kWhite);
    }
    /** Chooses a random shade of the color of your choosing. */
    public RandomLEDPattern withSingleColor(Color shade)
    {
        return withGradient(Color.kBlack, shade);
    }
    /** Chooses a random color between two of your choosing. */
    public RandomLEDPattern withGradient(Color colorA, Color colorB)
    {      
        isGradient = true;
        this.colorA = colorA;
        this.colorB = colorB;
        return this;
    }
    /** Sets a new gamma for color interpolation. */
    public RandomLEDPattern withGamma(double gamma)
    {
        this.gamma = gamma;
        return this;
    }
    /** Reset all pattern properties. */
    public RandomLEDPattern pureRandom()
    {
        // Chooses any possible color.
        isGradient = false;
        return this;
    }

    /** Is this pattern limited to shades of black and white? */
    public boolean isMono()
    {
        return isGradient &&
               ((colorA == Color.kBlack && colorB == Color.kWhite) ||
                (colorA == Color.kWhite && colorB == Color.kBlack));
    }
    /** Is this pattern limited to a single color? */
    public boolean isSingleColor()
    {
        return isGradient &&
               ((colorA == Color.kBlack && colorB != Color.kBlack) ||
                (colorA != Color.kBlack && colorB == Color.kBlack));
    }
    /** Is this pattern limited to a range of colors? */
    public boolean isGradient()
    {
        return isGradient;
    }
    /** Returns the gamma value the pattern will use for interpolation. */
    public double getGamma()
    {
        return gamma;
    }

    /** Pick a random color according to the rules set. */
    public Color get()
    {
        if (isGradient)
        {
            // Lerp between two.
            double factor = Math.random();

            // Factors in gamma. Raise each component to the gamma power,
            // then interpolate, and then apply the inverse power.
            // Makes the colors look better? Yes. Overly complex? Absolutely.
            double rGammaA = Math.pow(colorA.red, gamma),
                   gGammaA = Math.pow(colorA.green, gamma),
                   bGammaA = Math.pow(colorA.blue, gamma),
                   rGammaB = Math.pow(colorB.red, gamma),
                   gGammaB = Math.pow(colorB.green, gamma),
                   bGammaB = Math.pow(colorB.blue, gamma);

            // TODO: This could also be optimized.
            // Since the colors aren't changed per method call, these values
            // should be stored when the colors are set. `Math.pow` is an
            // expensive operation.

            double newR = rGammaA + factor * (rGammaB - rGammaA),
                   newG = gGammaA + factor * (gGammaB - gGammaA),
                   newB = bGammaA + factor * (bGammaB - bGammaA);

            // Inverse gamma.
            final double invGamma = 1 / gamma;
            newR = Math.pow(newR, invGamma);
            newG = Math.pow(newG, invGamma);
            newB = Math.pow(newB, invGamma);
                   
            return new Color(newR, newG, newB);
        }
        else
        {
            // Completely random color.
            return new Color(Math.random(), Math.random(), Math.random());
        }
    }

    @Override
    public void applyTo(LEDReader reader, LEDWriter writer)
    {
        int length = reader.getLength();
        for (int i = 0; i < length; i++)
        {
            Color color = get();
            writer.setRGB(i, (int)(255 * color.red),
                             (int)(255 * color.green),
                             (int)(255 * color.blue));
        }
    }
}
