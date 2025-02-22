package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase 
{

    public static final int kIntakeMotorId = 1;
    public static final double kCoralIntakeVoltage = 4;
    public static final double kAlgaeIntakeVoltage = 4;
    public static final double kAlgaeDefaultVoltage = 1;

    private boolean mIsIntaking;
    private boolean mIsOutTaking;
    private TalonFX mAlgaeIntake;
    private TalonFX mCoralIntake;

    public IntakeSubsystem()
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
        coralCurrentConfig.SupplyCurrentLimit = 30;
        coralCurrentConfig.SupplyCurrentLimitEnable = true;
        coralCurrentConfig.StatorCurrentLimitEnable = true;
        coralCurrentConfig.StatorCurrentLimit = 30;
        coralMotorConfig.NeutralMode = NeutralModeValue.Brake;
        coralConfig.withCurrentLimits(coralCurrentConfig);
        coralConfig.withMotorOutput(coralMotorConfig);
        mCoralIntake.getConfigurator().apply(coralConfig);
    }

    public void intakeCoral()
    {
        mCoralIntake.setVoltage(kCoralIntakeVoltage);
        mIsIntaking = true;
        mIsOutTaking = false;

    }

    public void outtakeCoral()
    {
        mCoralIntake.setVoltage(-kCoralIntakeVoltage);
        mIsIntaking = false;
        mIsOutTaking = true;
    }

    public void intakeAlgae()
    {
        mAlgaeIntake.setVoltage(kAlgaeIntakeVoltage);
        mIsIntaking = true;
        mIsOutTaking = false;

    }

    public void outtakeAlgae()
    {
        mAlgaeIntake.setVoltage(-kAlgaeIntakeVoltage);
        mIsIntaking = false;
        mIsOutTaking = true;
    }

    public void stop()
    {
        mCoralIntake.stopMotor();
        mAlgaeIntake.setVoltage(kAlgaeDefaultVoltage);
        mIsIntaking = false;
        mIsOutTaking = false;
    }

    public boolean isIntaking()
    {
        return mIsIntaking;
    }

    public boolean isOutTaking()
    {
        return mIsOutTaking;
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Algae Intake Current", mAlgaeIntake.getSupplyCurrent().getValueAsDouble());
    }
}
