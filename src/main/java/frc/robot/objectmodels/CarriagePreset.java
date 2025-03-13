package frc.robot.objectmodels;

import frc.robot.subsystems.ElevatorCarriageSubsystem;

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
    public static final CarriagePreset kCoralL1 = new CarriagePreset(67, -71, 0.036);
    public static final CarriagePreset kCoralL2 = new CarriagePreset(89, -114, 10.316); // +2.5in
    public static final CarriagePreset kCoralL3 = new CarriagePreset(89, -114, 28.125); // +2.5in
    public static final CarriagePreset kCoralL4 = new CarriagePreset(90, -102, 52.625);
    public static final CarriagePreset kCoralLoad = new CarriagePreset(95, -50, 6.647);
    public static final CarriagePreset kStowCoral = new CarriagePreset(65, 0, 0); // Also used to be known as "moving" in some places of the code.
    
    public static final CarriagePreset kAlgaeBarge = new CarriagePreset(90, -100, 52.125);
    public static final CarriagePreset kAlgaeProcessor = new CarriagePreset(20, -110, 0);
    public static final CarriagePreset kAlgaeGround = new CarriagePreset(-45, -90, 12.125);
    public static final CarriagePreset kAlgaeL2 = new CarriagePreset(40, -120, 12.125);
    public static final CarriagePreset kAlgaeL3 = new CarriagePreset(40, -120, 30.469);
    public static final CarriagePreset kStowAlgae = new CarriagePreset(65, -60, 0);

    public static final CarriagePreset kAway = new CarriagePreset(0, 0, 0);
    public static final CarriagePreset kClimb = new CarriagePreset(85, 25, 0);

    public final double kArmPreset, kWristPreset, kElevatorPreset;

    public CarriagePreset(double pArmPreset, double pWristPreset, double pElevatorPreset)
    {
        kArmPreset = pArmPreset;
        kWristPreset = pWristPreset;
        kElevatorPreset = pElevatorPreset + ElevatorCarriageSubsystem.initialHeight;
    }

    /**
     * Returns a NEW preset with the same arm & wrist positions as the current preset,
     * but with a new elevator position.
     */
    public CarriagePreset withElevatorPreset(double elevatorPreset)
    {
        return new CarriagePreset(kArmPreset, kWristPreset, elevatorPreset);
    }
    /**
     * Returns a NEW preset with the same arm & wrist positions as the current preset,
     * but with the elevator position from the variable.
     */
    public CarriagePreset withElevatorPreset(CarriagePreset elevatorPreset)
    {
        return new CarriagePreset(kArmPreset, kWristPreset, elevatorPreset.kElevatorPreset);
    }

    /**
     * Returns a NEW preset with the same elevator position as the current preset,
     * but with new arm & wrist positions.
     */
    public CarriagePreset withArmWristPreset(double armPreset, double wristPreset)
    {
        return new CarriagePreset(armPreset, wristPreset, kElevatorPreset);
    }
    /**
     * Returns a NEW preset with the same elevator position as the current preset,
     * but with the arm & wrist positions from the variable.
     */
    public CarriagePreset withArmWristPreset(CarriagePreset armWristPreset)
    {
        return new CarriagePreset(armWristPreset.kArmPreset, armWristPreset.kWristPreset, kElevatorPreset);
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
