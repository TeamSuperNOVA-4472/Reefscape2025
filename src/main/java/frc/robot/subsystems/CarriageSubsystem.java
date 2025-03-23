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
import frc.robot.objectmodels.CarriagePreset;

public class CarriageSubsystem extends SubsystemBase 
{
    public static final CarriageSubsystem kInstance = new CarriageSubsystem();

    // Motor IDs    
    public static final int armMotorId = 22;
    public static final int wristMotorId = 20;

    // Switch ports. FIXME: These aren't used, do we need them?
    public static final int armBottomSwitch = 1;
    public static final int armTopSwitch = 1;
    public static final int wristSwitch = 1;

    // PID Constants for Arm
    public static final double armP = 0.075;
    public static final double armI = 0;
    public static final double armD = 0.005;
    private static final double armKG = 0.44;

    // PID Constants for Wrist
    public static final double wristP = 0.1;
    public static final double wristI = 0;
    public static final double wristD = 0;
    private static final double wristKG = 0.3;
    
    // Shift the numbers so "zero" is reasonable.
    // For the arm, "zero" means the arm is perpendicular to the elevator. Higher numbers move it up.
    // For the wrist, "zero" means the wrist is PARALLEL to the ARM. Higher numbers move it up.
    private static final double armOffset = 150;
    private static final double wristOffset = 122;

    // If the arm's rotation is greater than this number, subtract a full rotation.
    // Helps with
    private static final double angleOverflowMin = 235;

    private TalonFX armMotor;
    private TalonFX wristMotor;

    private final DutyCycleEncoder mWristEncoder;
    private final DutyCycleEncoder mElbowEncoder;

    private ProfiledPIDController armPID;
    private ProfiledPIDController wristPID;
    
    private Optional<CarriagePreset> carriagePreset = Optional.empty();

    boolean algaeMode = false;

    private CarriageSubsystem() 
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
        carriagePreset = Optional.empty();
        armMotor.stopMotor();
        wristMotor.stopMotor();
    }

    public void setArmVoltage(double voltage) 
    {
        armMotor.setVoltage(voltage + armKG * Math.cos(Math.toRadians(this.getArmAngle())));
    }
    
    public void setManualArmVoltage(double voltage) 
    {
        carriagePreset = Optional.empty();
        armMotor.setVoltage(voltage + armKG * Math.cos(Math.toRadians(this.getArmAngle())));
    }

    public void setWristVoltage(double voltage) 
    {
        wristMotor.setVoltage(voltage + wristKG * Math.cos(Math.toRadians(this.getAbsoluteWristAngle())));
    }

    public void setManualWristVoltage(double voltage) 
    {
        carriagePreset = Optional.empty();
        wristMotor.setVoltage(voltage + wristKG * Math.cos(Math.toRadians(this.getAbsoluteWristAngle())));
    }

    public void setPreset(CarriagePreset preset)
    {
        carriagePreset = Optional.of(preset);
    }

    // FIXME: getAlgaeMode() appears to be identical in function to IntakeSubsystem.hasAlgae().
    //        Is this needed?
    public boolean getAlgaeMode()
    {
        return algaeMode;
    }
    public void setAlgaeMode(boolean newMode)
    {
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

    public Optional<CarriagePreset> getActivePreset()
    {
        return carriagePreset;
    }

    public double getArmSetpoint()
    {
        // If a preset is set, that's its "setpoint."
        if (carriagePreset.isPresent()) return carriagePreset.get().kArmPreset;
        else return 0.0;
    }
    public double getWristSetpoint() 
    {
        // If a preset is set, that's its "setpoint."
        if (carriagePreset.isPresent()) return carriagePreset.get().kWristPreset;
        else return 0.0;
    }

    public double getArmAngle()
    {
        return mElbowEncoder.get() * 360 - armOffset;
    }

    public double getWristAngle()
    {
        double wristAngle = mWristEncoder.get() * 360;
        
        // Compensate for values above our expected range.
        // If the true angle is above our range, subtract a full rotation.
        if (wristAngle > angleOverflowMin) wristAngle -= 360;
        return wristAngle - wristOffset;
    }

    public double getAbsoluteWristAngle()
    {
        return this.getWristAngle() + this.getArmAngle();
    }

    public void resetWristPID()
    {
        wristPID.reset(getWristAngle());
    }
    public void resetArmPID()
    {
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
        // Return the "desired" position, not the true one.
        // So if we don't have a desired preset, return a default value.
        if (carriagePreset.isEmpty()) return -100;
        
        double armX = 13 * Math.cos(Math.toRadians(getArmSetpoint()));
        double wristX = -10 * Math.cos(Math.toRadians(getWristSetpoint() + getArmSetpoint())) + -6 * Math.sin(Math.toRadians(getWristSetpoint() + getArmSetpoint()));
        //SmartDashboard.putNumber("GetTargetX Value", armX + wristX);
        return armX + wristX;
    }

    @Override
    public void periodic() 
    {
        SmartDashboard.putNumber("Elbow Angle", this.getArmAngle());
        SmartDashboard.putNumber("Wrist Angle", this.getWristAngle());
        SmartDashboard.putNumber("Absolute Wrist Angle", this.getAbsoluteWristAngle());

        if (carriagePreset.isEmpty()) return; // No preset.
        CarriagePreset preset = carriagePreset.get();

        double armCurrentPosition = this.getArmAngle();
        double armSpeed = armPID.calculate(armCurrentPosition, preset.kArmPreset);
        setArmVoltage(armSpeed);

        double wristCurrentPosition = this.getWristAngle();
        double wristSpeed = wristPID.calculate(wristCurrentPosition, preset.kWristPreset);
        setWristVoltage(wristSpeed);
    }
}
