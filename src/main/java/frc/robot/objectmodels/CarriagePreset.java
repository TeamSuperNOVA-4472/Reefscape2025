package frc.robot.objectmodels;

/**
 * Represets a position for the carriage (arm, wrist, and elevator) to be in.
 * Thanks to Sophia for coming up with this.
 * 
 * You can use the static fields in this class for easy-to-use presets,
 * or you can use the constructor if you need to make your own.
 */
public class CarriagePreset
{
    // Actual presets go here.
    public static final CarriagePreset kCoralL1 = new CarriagePreset(67, -71, 12.911);
    public static final CarriagePreset kCoralL2 = new CarriagePreset(67, -71, 21.691);
    public static final CarriagePreset kCoralL3 = new CarriagePreset(67, -71, 38.5);
    public static final CarriagePreset kCoralL4 = new CarriagePreset(90, -102, 65.5);
    public static final CarriagePreset kCoralLoad = new CarriagePreset(95, -50, 19.522);
    public static final CarriagePreset kStowCoral = new CarriagePreset(65, 0, 12.875); // Also used to be known as "moving" in some places of the code.
    
    public static final CarriagePreset kAlgaeBarge = new CarriagePreset(90, -100, 65);
    public static final CarriagePreset kAlgaeProcessor = new CarriagePreset(20, -110, 12.875);
    public static final CarriagePreset kAlgaeGround = new CarriagePreset(-45, -90, 25);
    public static final CarriagePreset kAlgaeL2 = new CarriagePreset(40, -120, 25);
    public static final CarriagePreset kAlgaeL3 = new CarriagePreset(40, -120, 43.344);
    public static final CarriagePreset kStowAlgae = new CarriagePreset(65, -60, 12.875);

    public static final CarriagePreset kAway = new CarriagePreset(0, 0, 12.875);
    public static final CarriagePreset kClimb = new CarriagePreset(85, 25, 12.875);

    public final double kArmPreset, kWristPreset, kElevatorPreset;

    public CarriagePreset(double pArmPreset, double pWristPreset, double pElevatorPreset)
    {
        kArmPreset = pArmPreset;
        kWristPreset = pWristPreset;
        kElevatorPreset = pElevatorPreset;
    }

    @Override
    public boolean equals(Object other)
    {
        if (this.getClass() != other.getClass()) return false; // Different types, obviously not a preset.

        // The other object is a preset, cast it as such and compare the values.
        CarriagePreset otherPreset = (CarriagePreset)other;
        return kArmPreset == otherPreset.kArmPreset &&
               kWristPreset == otherPreset.kWristPreset &&
               kElevatorPreset == otherPreset.kElevatorPreset;
    }
}
