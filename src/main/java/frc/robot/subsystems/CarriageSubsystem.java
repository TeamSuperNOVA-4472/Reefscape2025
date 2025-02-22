package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CarriageSubsystem extends SubsystemBase 
{

    public static final int armMotorId = 22;

    public static final int wristMotorId = 20;

    public static final int armBottomSwitch = 1;

    public static final int armTopSwitch = 1;

    public static final int wristSwitch = 1;

    public static final double armP = 0.075;

    public static final double armI = 0;

    public static final double armD = 0.005;

    public static final double wristP = 0.1;

    public static final double wristI = 0;

    public static final double wristD = 0;

    public static final double armPresetKAway = 0.0;

    public static final double armPresetGroundPickup = 1.0;

    public static final double armPresetL1 = 67;

    public static final double armPresetL2 = 67;

    public static final double armPresetL3 = 67;

    public static final double armPresetL4 = 67; // FIXME: WE'RE COOKED

    private static final double armOffset = 150;

    private static final double wristOffset = 122;

    public static final double wristPresetKAway = 0.0;

    public static final double wristPresetGroundPickup = 1.0;

    public static final double wristPresetL1 = -71;

    public static final double wristPresetL2 = -71;

    public static final double wristPresetL3 = -71;

    public static final double wristPresetL4 = -71; // FIXME: WE'RE COOKED

    private static final double armKG = 0.44;

    private static final double wristKG = 0.3;

    private static final double angleOverflowMin = 235;

    private static final double maxWristVoltage = 4;

    private TalonFX armMotor;

    private TalonFX wristMotor;

    private final DutyCycleEncoder mWristEncoder;
    private final DutyCycleEncoder mElbowEncoder;

    private Optional<Double> wristPreset = Optional.empty();
    private Optional<Double> elbowPreset = Optional.empty();

    private ProfiledPIDController armPID;

    private ProfiledPIDController wristPID;

    public CarriageSubsystem() 
    {
        armMotor = new TalonFX(armMotorId, "CANivore");
        wristMotor = new TalonFX(wristMotorId, "CANivore");

        TalonFXConfiguration armConfig = new TalonFXConfiguration();
        CurrentLimitsConfigs armCurrentConfig = new CurrentLimitsConfigs();
        MotorOutputConfigs armMotorConfig = new MotorOutputConfigs();
        armMotor.getConfigurator().refresh(armConfig);
        armMotor.getConfigurator().refresh(armCurrentConfig);
        armMotor.getConfigurator().refresh(armMotorConfig);
        armCurrentConfig.SupplyCurrentLimit = 60;
        armCurrentConfig.SupplyCurrentLimitEnable = true;
        armCurrentConfig.StatorCurrentLimitEnable = true;
        armCurrentConfig.StatorCurrentLimit = 60;
        armMotorConfig.NeutralMode = NeutralModeValue.Brake;
        armConfig.withCurrentLimits(armCurrentConfig);
        armConfig.withMotorOutput(armMotorConfig);
        armMotor.getConfigurator().apply(armConfig);

        TalonFXConfiguration wristConfig = new TalonFXConfiguration();
        CurrentLimitsConfigs wristCurrentConfig = new CurrentLimitsConfigs();
        MotorOutputConfigs wristMotorConfig = new MotorOutputConfigs();
        wristMotor.getConfigurator().refresh(wristConfig);
        wristMotor.getConfigurator().refresh(wristCurrentConfig);
        wristMotor.getConfigurator().refresh(wristMotorConfig);
        wristCurrentConfig.SupplyCurrentLimit = 20;
        wristCurrentConfig.SupplyCurrentLimitEnable = true;
        wristCurrentConfig.StatorCurrentLimitEnable = true;
        wristCurrentConfig.StatorCurrentLimit = 20;
        wristMotorConfig.NeutralMode = NeutralModeValue.Brake;
        wristConfig.withCurrentLimits(wristCurrentConfig);
        wristConfig.withMotorOutput(wristMotorConfig);
        wristMotor.getConfigurator().apply(wristConfig);

        armPID = new ProfiledPIDController(armP, armI, armD, new Constraints(720, 720));
        wristPID = new ProfiledPIDController(wristP, wristI, wristD, new Constraints(360, 360));

        mElbowEncoder = new DutyCycleEncoder(0); 
        mWristEncoder = new DutyCycleEncoder(1);
    }

    public void stop()
    {
        elbowPreset = Optional.empty();
        wristPreset = Optional.empty();
        armMotor.stopMotor();
        wristMotor.stopMotor();
    }

    public void setArmVoltage(double voltage) 
    {
        armMotor.setVoltage(voltage + armKG * Math.cos(Math.toRadians(this.getArmAngle())));
    }
    
    public void setManualArmVoltage(double voltage) 
    {
        elbowPreset = Optional.empty();
        armMotor.setVoltage(voltage + armKG * Math.cos(Math.toRadians(this.getArmAngle())));
    }

    public void setWristVoltage(double voltage) 
    {
        wristMotor.setVoltage(voltage + wristKG * Math.cos(Math.toRadians(this.getAbsoluteWristAngle())));
    }

    public void setManualWristVoltage(double voltage) 
    {
        wristPreset = Optional.empty();
        wristMotor.setVoltage(voltage + wristKG * Math.cos(Math.toRadians(this.getAbsoluteWristAngle())));
    }
    /*public boolean isArmAtBottom() 
    {
        return armBottom.get();
    }
    public boolean isArmAtTop() 
    {
        return armTop.get();
    }

    public boolean isWristAtLimit() 
    {
        return wristLimit.get();
    }*/

    public void setArmPreset(Double armPreset)
    {
        elbowPreset = Optional.of(armPreset);
    }
    public void setWristPreset(Double wristPre)
    {
        wristPreset = Optional.of(wristPre);
    }

    public double getArmCurrentPosition() 
    {
        return armMotor.getPosition().getValueAsDouble();
    }
    public double getWristCurrentPosition() 
    {
        return wristMotor.getPosition().getValueAsDouble();
    }

    public double getArmSetpoint() 
    {
        return elbowPreset.orElse(0.0); //TODO: Fix or remove.
    }
    public double getWristSetpoint() 
    {
        return wristPreset.orElse(0.0); //TODO: Fix or remove.
    }

    public double getArmAngle(){
        return mElbowEncoder.get()*360 - armOffset;
    }

    public double getWristAngle(){
        double wristAngle = mWristEncoder.get()*360;
        if (wristAngle > angleOverflowMin){
            wristAngle -= 360;
        }
        return wristAngle - wristOffset;
    }

    public double getAbsoluteWristAngle(){
        return this.getWristAngle() + this.getArmAngle();
    }

    public void resetWristPID(){
        wristPID.reset(getWristAngle());
    }

    public void resetArmPID(){
        armPID.reset(getArmAngle());
    }

    @Override
    public void periodic() 
    {
        SmartDashboard.putNumber("Elbow Angle", this.getArmAngle());
        SmartDashboard.putNumber("Wrist Angle", this.getWristAngle());
        SmartDashboard.putNumber("Absolute Wrist Angle", this.getAbsoluteWristAngle());

        if (elbowPreset.isEmpty() || wristPreset.isEmpty()) return; // No preset.

        double armCurrentPosition = this.getArmAngle();
        double armSpeed = armPID.calculate(armCurrentPosition, elbowPreset.get());
        this.setArmVoltage(armSpeed);
        double wristCurrentPosition = this.getWristAngle();
        double wristSpeed = wristPID.calculate(wristCurrentPosition, wristPreset.get());
        this.setWristVoltage(wristSpeed);
    }
}
