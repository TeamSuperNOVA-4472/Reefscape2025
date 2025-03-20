package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase 
{
    public static final IntakeSubsystem kInstance = new IntakeSubsystem();

    public static final int kIntakeMotorId = 1;
    public static final double kCoralIntakeVoltage = 6;
    public static final double kCoralOuttakeVoltage = 8;
    public static final double kCoralDefaultVoltage = 1;
    public static final double kAlgaeIntakeVoltage = 6;
    public static final double kAlgaeOuttakeVoltage = 12;
    public static final double kAlgaeDefaultVoltage = 1;
    public static final double kAlgaeCurrentThreshold = 2;

    private double algaeTargetVoltage = kAlgaeDefaultVoltage;

    private boolean hasCoral;
    private TalonFX mAlgaeIntake;
    private TalonFX mCoralIntake;

    private IntakeSubsystem()
    {
        mCoralIntake = new TalonFX(23, "CANivore");
        mAlgaeIntake = new TalonFX(24, "CANivore");
        
        TalonFXConfiguration algaeConfig = new TalonFXConfiguration();
        CurrentLimitsConfigs algaeCurrentConfig = new CurrentLimitsConfigs();
        MotorOutputConfigs algaeMotorConfig = new MotorOutputConfigs();
        mAlgaeIntake.getConfigurator().refresh(algaeConfig);
        mAlgaeIntake.getConfigurator().refresh(algaeCurrentConfig);
        mAlgaeIntake.getConfigurator().refresh(algaeMotorConfig);
        algaeCurrentConfig.SupplyCurrentLimit = 30;
        algaeCurrentConfig.SupplyCurrentLimitEnable = true;
        algaeCurrentConfig.StatorCurrentLimitEnable = true;
        algaeCurrentConfig.StatorCurrentLimit = 30;
        algaeMotorConfig.NeutralMode = NeutralModeValue.Brake;
        algaeConfig.withCurrentLimits(algaeCurrentConfig);
        algaeConfig.withMotorOutput(algaeMotorConfig);
        mAlgaeIntake.getConfigurator().apply(algaeConfig);

        TalonFXConfiguration coralConfig = new TalonFXConfiguration();
        CurrentLimitsConfigs coralCurrentConfig = new CurrentLimitsConfigs();
        MotorOutputConfigs coralMotorConfig = new MotorOutputConfigs();
        mCoralIntake.getConfigurator().refresh(coralConfig);
        mCoralIntake.getConfigurator().refresh(coralCurrentConfig);
        mCoralIntake.getConfigurator().refresh(coralMotorConfig);
        coralCurrentConfig.SupplyCurrentLimit = 20;
        coralCurrentConfig.SupplyCurrentLimitEnable = true;
        coralCurrentConfig.StatorCurrentLimitEnable = true;
        coralCurrentConfig.StatorCurrentLimit = 20;
        coralMotorConfig.NeutralMode = NeutralModeValue.Brake;
        coralConfig.withCurrentLimits(coralCurrentConfig);
        coralConfig.withMotorOutput(coralMotorConfig);
        mCoralIntake.getConfigurator().apply(coralConfig);
        hasCoral = false;
    }

    public void intakeCoral()
    {
        mCoralIntake.setVoltage(kCoralIntakeVoltage);
    }

    public void outtakeCoral()
    {
        mCoralIntake.setVoltage(-kCoralOuttakeVoltage);
    }

    public void intakeAlgae()
    {
        mAlgaeIntake.setVoltage(-kAlgaeIntakeVoltage);
        algaeTargetVoltage = -kAlgaeIntakeVoltage;

    }

    public void outtakeAlgae()
    {
        mAlgaeIntake.setVoltage(kAlgaeOuttakeVoltage);
        algaeTargetVoltage = kAlgaeOuttakeVoltage;
    }

    public void stopCoral()
    {
        mCoralIntake.setVoltage(kCoralDefaultVoltage);
    }
    public void stopAlgae()
    {
        mAlgaeIntake.setVoltage(-kAlgaeDefaultVoltage);
        algaeTargetVoltage = kAlgaeDefaultVoltage;
    }

    /**
     * Stops both the algae intake/outtake and the coral intake/outtake at the same time.
     * Just a shorthand for calling stopCoral() and stopAlgae().
     */
    public void stopBoth()
    {
        stopCoral();
        stopAlgae();
    }

    public boolean hasAlgae(){
        return Math.abs(mAlgaeIntake.getVelocity().getValueAsDouble()) < 1 && Math.abs(mAlgaeIntake.getStatorCurrent().getValueAsDouble()) > 20;
    }

    public boolean hasCoral(){
        return Math.abs(mCoralIntake.getStatorCurrent().getValueAsDouble()) > 15;
    }

    public void setCoral(boolean newCoral){
        hasCoral = newCoral;
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Algae Intake Current", mAlgaeIntake.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Algae Target Voltage", algaeTargetVoltage);
        SmartDashboard.putNumber("Algae Actual Voltage", mAlgaeIntake.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Algae Actual Velocity", mAlgaeIntake.getVelocity().getValueAsDouble());
        SmartDashboard.putBoolean("Algae in Intake", hasAlgae());
        SmartDashboard.putBoolean("Has Coral", hasCoral());
        /*if (Math.abs(mCoralIntake.getMotorVoltage().getValueAsDouble()) > 1 && Math.abs(mCoralIntake.getVelocity().getValueAsDouble()) < 0.2){
            hasCoral = true;
        }*/
    }
}
