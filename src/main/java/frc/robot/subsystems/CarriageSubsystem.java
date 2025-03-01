package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
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

    public static final double armMovingAngle = 65;

    public static final double armCoralLoad = 97;
    public static final double wristCoralLoad = -57.3;

    public static final double armAlgaeLoad = 67;

    public static final double armPresetL1 = 67;

    public static final double armPresetL2 = 67;

    public static final double armPresetL3 = 67;

    public static final double armPresetL4 = 90;

    private static final double armOffset = 150;

    private static final double wristOffset = 122;

    public static final double wristPresetKAway = 0.0;

    public static final double wristPresetGroundPickup = 1.0;

    public static final double wristMovingAngle = 0;

    public static final double wristAlgaeLoad = -90;

    public static final double wristPresetL1 = -71;

    public static final double wristPresetL2 = -71;

    public static final double wristPresetL3 = -71;

    public static final double wristPresetL4 = -102;

    public static final double wristPresetMoving = 0;

    public static final double armPresetMoving = 60;

    private static final double armKG = 0.44;

    private static final double wristKG = 0.3;

    private static final double angleOverflowMin = 235;

    private static final double maxWristVoltage = 4;

    public static final double armAlgaeBarge = 90;
    public static final double armAlgaeProcessor = 20;
    public static final double armAlgaeGround = -45;
    public static final double armAlgaeL2 = 40;
    public static final double armAlgaeL3 = 40;
    public static final double armAlgaeStow = 60;

    public static final double wristAlgaeBarge = -100;
    public static final double wristAlgaeProcessor = -110;
    public static final double wristAlgaeGround = -90;
    public static final double wristAlgaeL2 = -120;
    public static final double wristAlgaeL3 = -120;
    public static final double wristAlgaeStow = -60;

    public static final double armClimb = 85;
    public static final double wristClimb = 25;

    private TalonFX armMotor;

    private TalonFX wristMotor;

    private final DutyCycleEncoder mWristEncoder;
    private final DutyCycleEncoder mElbowEncoder;

    private Optional<Double> wristPreset = Optional.empty();
    private Optional<Double> elbowPreset = Optional.empty();

    private ProfiledPIDController armPID;

    private ProfiledPIDController wristPID;

    boolean algaeMode = false;

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
        wristCurrentConfig.SupplyCurrentLimit = 25;
        wristCurrentConfig.SupplyCurrentLimitEnable = true;
        wristCurrentConfig.StatorCurrentLimitEnable = true;
        wristCurrentConfig.StatorCurrentLimit = 25;
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

    public boolean getAlgaeMode(){
        return algaeMode;
    }
    public void setAlgaeMode(boolean newMode){
        algaeMode = newMode;
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
        return elbowPreset.orElse(0.0);
    }
    public double getWristSetpoint() 
    {
        return wristPreset.orElse(0.0);
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

    public double getCarriageX()
    {
        double armX = 13 * Math.cos(Math.toRadians(getArmAngle()));
        double wristX = -10 * Math.cos(Math.toRadians(getAbsoluteWristAngle())) + -6 * Math.sin(Math.toRadians(getAbsoluteWristAngle()));
        return armX + wristX;
    }

    public double getCarriageTargetX()
    {
        if (elbowPreset.isEmpty() || wristPreset.isEmpty())
        {
            return -100;
        }
        
        double armX = 13 * Math.cos(Math.toRadians(getArmSetpoint()));
        double wristX = -10 * Math.cos(Math.toRadians(getWristSetpoint() + getArmSetpoint())) + -6 * Math.sin(Math.toRadians(getWristSetpoint() + getArmSetpoint()));
        SmartDashboard.putNumber("GetTargetX Value", armX + wristX);
        return armX + wristX;
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
