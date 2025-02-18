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
import frc.robot.objectmodels.IntakePresets;

public class CarriageSubsystem extends SubsystemBase 
{

    public static final int armMotorId = 22;

    public static final int wristMotorId = 20;

    public static final int armBottomSwitch = 1;

    public static final int armTopSwitch = 1;

    public static final int wristSwitch = 1;

    public static final double armP = 0.05;

    public static final double armI = 0;

    public static final double armD = 0.005;

    public static final double wristP = 0.1;

    public static final double wristI = 0;

    public static final double wristD = 0;

    private static final double armPresetKAway = 0.0;

    private static final double armPresetGroundPickup = 1.0;

    private static final double armPresetL1 = 2.0;

    private static final double armPresetL2 = 3.0;

    private static final double armPresetL3 = 4.0;

    private static final double armPresetL4 = 4.0;

    private static final double armOffset = 150;

    private static final double wristOffset = 122;

    private static final double wristPresetKAway = 0.0;

    private static final double wristPresetGroundPickup = 1.0;

    private static final double wristPresetL1 = 1.0;

    private static final double wristPresetL2 = 1.0;

    private static final double wristPresetL3 = 1.0;

    private static final double wristPresetL4 = 1.0;

    private static final double armKG = 0.35;

    private static final double wristKG = 0.3;

    private static final double angleOverflowMin = 235;

    private static final double maxWristVoltage = 4;

    private TalonFX armMotor;

    private TalonFX wristMotor;

    private final DutyCycleEncoder mWristEncoder;
    private final DutyCycleEncoder mElbowEncoder;

    private Optional<IntakePresets> activePreset = Optional.empty();

    private PIDController armPID;

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

        armPID = new PIDController(armP, armI, armD);
        wristPID = new ProfiledPIDController(wristP, wristI, wristD, new Constraints(360, 360));

        mElbowEncoder = new DutyCycleEncoder(0); 
        mWristEncoder = new DutyCycleEncoder(1); 
    }

    public void stop()
    {
        activePreset = Optional.empty();
        armMotor.stopMotor();
        wristMotor.stopMotor();
    }

    public void setArmVoltage(double voltage) 
    {
        armMotor.setVoltage(voltage + armKG * Math.cos(Math.toRadians(this.getArmAngle())));
    }
    
    public void setManualArmVoltage(double voltage) 
    {
        activePreset = Optional.empty();
        armMotor.setVoltage(voltage + armKG * Math.cos(Math.toRadians(this.getArmAngle())));
    }

    public void setWristVoltage(double voltage) 
    {
        wristMotor.setVoltage(voltage + wristKG * Math.cos(Math.toRadians(this.getAbsoluteWristAngle())));
    }

    public void setManualWristVoltage(double voltage) 
    {
        activePreset = Optional.empty();
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

    public void setActivePreset(IntakePresets preset)
    {
        activePreset = Optional.of(preset);
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
        return armPID.getSetpoint();
    }
    public double getWristSetpoint() 
    {
        return 0;
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

    @Override
    public void periodic() 
    {
        SmartDashboard.putNumber("Elbow Angle", this.getArmAngle());
        SmartDashboard.putNumber("Wrist Angle", this.getWristAngle());
        SmartDashboard.putNumber("Absolute Wrist Angle", this.getAbsoluteWristAngle());

        if (activePreset.isEmpty()) return; // No preset.
        /*else
        {
            switch (activePreset.get())
            {
                case kAway:
                    armPID.setSetpoint(armPresetKAway);
                    wristPID.setSetpoint(wristPresetKAway);
                    break;

                case kGroundPickup:
                    armPID.setSetpoint(armPresetGroundPickup);
                    wristPID.setSetpoint(wristPresetGroundPickup);
                    break;

                case kScoreL1:
                    armPID.setSetpoint(armPresetL1);
                    wristPID.setSetpoint(wristPresetL1);
                    break;

                case kScoreL2:
                    armPID.setSetpoint(armPresetL2);
                    wristPID.setSetpoint(wristPresetL2);
                    break;

                case kScoreL3:
                    armPID.setSetpoint(armPresetL3);
                    wristPID.setSetpoint(wristPresetL3);
                    break;

                case kScoreL4:
                    armPID.setSetpoint(armPresetL4);
                    wristPID.setSetpoint(wristPresetL4);
                    break;

                default: return; // Unknown preset. Shouldn't happen.
            }
        }*/

        double armCurrentPosition = this.getArmAngle();
        double armSpeed = armPID.calculate(armCurrentPosition, 70);
        this.setArmVoltage(0);
        //SmartDashboard.putNumber("Arm Current", armMotor.getSupplyCurrent().getValueAsDouble());
        double wristCurrentPosition = this.getWristAngle();
        double wristSpeed = wristPID.calculate(wristCurrentPosition, -90);
        //wristSpeed = MathUtil.clamp(wristSpeed, -maxWristVoltage, maxWristVoltage);
        this.setWristVoltage(wristSpeed);
    }
}
