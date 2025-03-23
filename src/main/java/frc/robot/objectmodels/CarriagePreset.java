package frc.robot.objectmodels;

import static frc.robot.subsystems.SwerveSubsystem.kA;

import java.util.ArrayList;

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
    public static final CarriagePreset kCoralL1 = new CarriagePreset(67, -61, 2);
    public static final CarriagePreset kCoralL2 = new CarriagePreset(89, -122, 14.75, true);
    public static final CarriagePreset kCoralL3 = new CarriagePreset(81, -122, 33.625, true);
    public static final CarriagePreset kCoralL4 = new CarriagePreset(100, -142, 60.23);
    public static final CarriagePreset kCoralLoad = new CarriagePreset(104, -71, 6.8);
    public static final CarriagePreset kStowCoral = new CarriagePreset(82, 10, 0); // Also used to be known as "moving" in some places of the code.
    
    public static final double kElevatorL2L3Intermediate = ElevatorCarriageSubsystem.initialHeight + 2; // The temporary place for L2 and L3.

    public static final CarriagePreset kAlgaeBarge = new CarriagePreset(90, -130, 60.23);
    public static final CarriagePreset kAlgaeProcessor = new CarriagePreset(20, -110, 0, true);
    public static final CarriagePreset kAlgaeGround = new CarriagePreset(-45, -90, 12.855, false);
    public static final CarriagePreset kAlgaeL2 = new CarriagePreset(40, -120, 12.855, true);
    public static final CarriagePreset kAlgaeL3 = new CarriagePreset(40, -120, 28.73, true);
    public static final CarriagePreset kStowAlgae = new CarriagePreset(65, -60, 0);

    public static final CarriagePreset kAway = new CarriagePreset(0, 0, 0);
    public static final CarriagePreset kClimb = new CarriagePreset(85, 25, 0);

    public final double kArmPreset, kWristPreset, kElevatorPreset;
    public final boolean kMoveArmFirst;

    public CarriagePreset(double pArmPreset, double pWristPreset, double pElevatorPreset)
    {
        this(pArmPreset, pWristPreset, pElevatorPreset, false);
    }
    public CarriagePreset(double pArmPreset, double pWristPreset, double pElevatorPreset, boolean pMoveArmFirst)
    {
        kArmPreset = pArmPreset;
        kWristPreset = pWristPreset;
        kElevatorPreset = pElevatorPreset + ElevatorCarriageSubsystem.initialHeight;
        kMoveArmFirst = pMoveArmFirst;
    }

    /**
     * Returns a NEW preset with the same arm & wrist positions as the current preset,
     * but with a new elevator position.
     */
    public CarriagePreset withElevatorPreset(double elevatorPreset)
    {
        return new CarriagePreset(kArmPreset, kWristPreset, elevatorPreset - ElevatorCarriageSubsystem.initialHeight);
    }
    /**
     * Returns a NEW preset with the same arm & wrist positions as the current preset,
     * but with the elevator position from the variable.
     */
    public CarriagePreset withElevatorPreset(CarriagePreset elevatorPreset)
    {
        return new CarriagePreset(kArmPreset, kWristPreset, elevatorPreset.kElevatorPreset - ElevatorCarriageSubsystem.initialHeight);
    }

    /**
     * Returns a NEW preset with the same arm & elevator positions as the current preset,
     * but with a new arm position.
     */
    public CarriagePreset withArmPreset(double armPreset)
    {
        return new CarriagePreset(armPreset, kWristPreset, kElevatorPreset - ElevatorCarriageSubsystem.initialHeight);
    }
    /**
     * Returns a NEW preset with the same arm & elevator positions as the current preset,
     * but with the arm position from the variable.
     */
    public CarriagePreset withArmPreset(CarriagePreset armPreset)
    {
        return new CarriagePreset(armPreset.kArmPreset, kWristPreset, kElevatorPreset - ElevatorCarriageSubsystem.initialHeight);
    }

    /**
     * Returns a NEW preset with the same arm & elevator positions as the current preset,
     * but with a new arm position.
     */
    public CarriagePreset withWristPreset(double wristPreset)
    {
        return new CarriagePreset(kArmPreset, wristPreset, kElevatorPreset - ElevatorCarriageSubsystem.initialHeight);
    }
    /**
     * Returns a NEW preset with the same arm & elevator positions as the current preset,
     * but with the arm position from the variable.
     */
    public CarriagePreset withWristPreset(CarriagePreset wristPreset)
    {
        return new CarriagePreset(kArmPreset, wristPreset.kWristPreset, kElevatorPreset - ElevatorCarriageSubsystem.initialHeight);
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

    @Override
    public String toString()
    {
        return "Arm: " + kArmPreset + "deg, Wrist: " + kWristPreset + "deg, Elev: " + kElevatorPreset + "in";
    }
}
