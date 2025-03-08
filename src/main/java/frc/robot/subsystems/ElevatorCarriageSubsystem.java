package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.objectmodels.CarriagePreset;

public class ElevatorCarriageSubsystem extends SubsystemBase
{
    public static final int kLeftElevatorMotorID = 0;

    public static final int kArmMotorId = 22;

    public static final int kWristMotorId = 20;

    public static final int kElevatorMotorId = 0;

    private final DutyCycleEncoder mElevatorEncoder;

    private final DutyCycleEncoder mWristEncoder;

    private final DutyCycleEncoder mElbowEncoder;

    private DigitalInput bottomSwitch;

    private DigitalInput topSwitch;

    public static final double initialHeight = 12.875;

    public static final double kPresetAway = initialHeight;

    public static final double kPresetGroundPickup = initialHeight;

    private static final double kElevatorP = 0.9;

    private static final double kElevatorI = 0.0;

    private static final double kElevatorD = 0.0;

    private static final double kArmP = 0.075;

    private static final double kArmI = 0;

    private static final double kArmD = 0.005;

    private static final double kWristP = 0.1;

    private static final double kWristI = 0;

    private static final double kWristD = 0;

    private static final double kArmKG = 0.44;

    private static final double kWristKG = 0.3;

    private static final double rotationsToInches = 0.45;

    private final TalonFX mElevatorMotor;

    private final TalonFX mArmMotor;

    private final TalonFX mWristMotor;

    private ProfiledPIDController elevatorPID;

    private ProfiledPIDController armPID;

    private ProfiledPIDController wristPID;

    private Optional<CarriagePreset> carriagePreset = Optional.empty();

    private boolean isMovingUp = false;

    private boolean isMovingDown = false;

    public ElevatorCarriageSubsystem()
    {
        mElevatorMotor = new TalonFX(kElevatorMotorId, "CANivore");

        mArmMotor = new TalonFX(kArmMotorId, "CANivore");

        mWristMotor = new TalonFX(kWristMotorId, "CANivore");

        TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();

        CurrentLimitsConfigs elevatorCurrentConfig = new CurrentLimitsConfigs();

        MotorOutputConfigs elevatorMotorConfig = new MotorOutputConfigs();

        mElevatorMotor.getConfigurator().refresh(elevatorConfig);

        mElevatorMotor.getConfigurator().refresh(elevatorCurrentConfig);

        mElevatorMotor.getConfigurator().refresh(elevatorMotorConfig);

        elevatorCurrentConfig.SupplyCurrentLimit = 30;

        elevatorCurrentConfig.SupplyCurrentLimitEnable = true;

        elevatorCurrentConfig.StatorCurrentLimitEnable = true;

        elevatorCurrentConfig.StatorCurrentLimit = 30;

        elevatorMotorConfig.Inverted = InvertedValue.Clockwise_Positive;

        elevatorMotorConfig.NeutralMode = NeutralModeValue.Brake;

        elevatorConfig.withCurrentLimits(elevatorCurrentConfig);

        elevatorConfig.withMotorOutput(elevatorMotorConfig);

        mElevatorMotor.getConfigurator().apply(elevatorConfig);

        mElevatorEncoder = new DutyCycleEncoder(0); 

        mElbowEncoder = new DutyCycleEncoder(0);

        mWristEncoder = new DutyCycleEncoder(0);

        elevatorPID = new ProfiledPIDController(kElevatorP, kElevatorI, kElevatorD, new Constraints(50, 50));

        armPID = new ProfiledPIDController(kArmP, kArmI, kArmD, new Constraints(720, 720));

        wristPID = new ProfiledPIDController(kWristP, kWristI, kWristD, new Constraints(360, 360));
    }

    public void stop()
    {
        carriagePreset = Optional.empty();

        mElevatorMotor.stopMotor();

        mArmMotor.stopMotor();

        mWristMotor.stopMotor();
    }

    public void setElevatorVoltage(double voltage)
    {
        mElevatorMotor.setVoltage(voltage);
    }

    public void setPreset(CarriagePreset preset)
    {
        carriagePreset = Optional.of(preset);
    }

    public double getElevatorPosition()
    {
        return mElevatorEncoder.get() * 360;
    }

    public double getArmAngle()
    {
        return mElbowEncoder.get() * 360.0;
    }

    public double getAbsoluteWristAngle()
    {
        return mWristEncoder.get() * 360.0;
    }

    private void moveElevator(CarriagePreset preset)
    {
        double currentHeight = getElevatorHeight();

        double elevatorOutput = elevatorPID.calculate(currentHeight, preset.kElevatorPreset);

        if (elevatorOutput > 0)
        {
            isMovingUp = true;

            isMovingDown = false;
        }
        
        else
        {
            isMovingDown = true;
            
            isMovingUp = false;
        }

        mElevatorMotor.set(elevatorOutput);
    }

    private void moveArm(CarriagePreset preset)
    {
        double currentAngle = getArmAngle();

        double armOutput = armPID.calculate(currentAngle, preset.kArmPreset);

        mArmMotor.set(armOutput);
    }

    private void moveWrist(CarriagePreset preset)
    {
        double currentWristAngle = getAbsoluteWristAngle();

        double wristOutput = wristPID.calculate(currentWristAngle, preset.kWristPreset);

        mWristMotor.set(wristOutput);
    }

    public double getElevatorHeight()
    {
        return mElevatorMotor.getPosition().getValueAsDouble() * rotationsToInches + initialHeight;
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Elevator Position", getElevatorPosition());

        if (carriagePreset.isPresent())
        {
            CarriagePreset preset = carriagePreset.get();

            moveElevator(preset);

            moveArm(preset);

            moveWrist(preset);
        }
    }
}
