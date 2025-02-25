package frc.robot.objectmodels;

public class IntakePreset {
    public static final IntakePreset kMoving = new IntakePreset(65, 0, 12.875);
    public static final IntakePreset kMovingAlgae = new IntakePreset(60, -60, 12.875);

    public static final IntakePreset kLoadAlgae = new IntakePreset(67, -90, 12.87);
    public static final IntakePreset kLoadCoralStage1 = new IntakePreset(65, -57.339534, 17.822144);
    public static final IntakePreset kLoadCoralStage2 = new IntakePreset(111.348181, -57.339534, 17.822144);

    public static final IntakePreset kScoreL1 = new IntakePreset(67, -71, 12.911);
    public static final IntakePreset kScoreL2 = new IntakePreset(67, -71, 20.191);
    public static final IntakePreset kScoreL3 = new IntakePreset(67, -71, 35.844);
    public static final IntakePreset kScoreL4 = new IntakePreset(90, -96, 65);

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