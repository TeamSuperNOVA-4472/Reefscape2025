package frc.robot.objectmodels;

public class IntakePreset {
    public static final IntakePreset kScoreL1 = new IntakePreset(67, -71, 12.911);
    public static final IntakePreset kScoreL2 = new IntakePreset(67, -71, 20.191);
    public static final IntakePreset kScoreL3 = new IntakePreset(67, -71, 35.844);

    public final double armPreset, wristPreset, elevatorPreset;

    public IntakePreset(double armPreset, double wristPreset, double elevatorPreset)
    {
        this.armPreset = armPreset;
        this.wristPreset = wristPreset;
        this.elevatorPreset = elevatorPreset;
    }

    @Override
    public boolean equals(Object obj) {
        if (obj.getClass() != this.getClass())
        {
            return false;
        }

        final IntakePreset preset = (IntakePreset) obj;
        return armPreset==preset.armPreset && wristPreset==preset.wristPreset && elevatorPreset==preset.elevatorPreset ? true : false;
    }
}