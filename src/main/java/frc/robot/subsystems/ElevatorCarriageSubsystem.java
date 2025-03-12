// IMPORTS
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
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.objectmodels.CarriagePreset;
import frc.robot.objectmodels.LightState;
import frc.robot.objectmodels.LightStatusRequest;

// ALL MAIN VARIABLES
public class ElevatorCarriageSubsystem extends SubsystemBase
{
    private final DigitalInput beamBreakSensor;

    private static ElevatorCarriageSubsystem kInstance = new ElevatorCarriageSubsystem();

    public static ElevatorCarriageSubsystem instance()
    {
        return kInstance;
    }

    public static final int kLeftElevatorMotorID = 0;

    public static final int kRightElevatorMotorID = 1;

    public static final int kArmMotorId = 20;

    public static final int kWristMotorId = 22;

    private final DutyCycleEncoder mWristEncoder;

    private final DutyCycleEncoder mArmEncoder;

    private DigitalInput bottomSwitch;

    private DigitalInput topSwitch;

    private LightStatusRequest lights;

    private static final double armOffset = 210;

    private static final double wristOffset = 130;

    private static final double angleOverflowMin = 200;

    public static final double initialHeight = 9.0;

    private static final double kElevatorP = 0.25;

    private static final double kElevatorI = 0.0;

    private static final double kElevatorD = 0.0;

    private static final double kElevatorKG = 0.2;

    private static final double kArmP = 0.075;

    private static final double kArmI = 0;

    private static final double kArmD = 0.005;

    private static final double kArmKG = 0.44;

    private static final double kWristP = 0.1;

    private static final double kWristI = 0;

    private static final double kWristD = 0;

    private static final double kWristKG = 0.3;

    private static final double rotationsToInches = 0.45*1.935*1.07;

    private final TalonFX mLeftElevatorMotor;

    private final TalonFX mRightElevatorMotor;

    private final TalonFX mArmMotor;

    private final TalonFX mWristMotor;

    private ProfiledPIDController elevatorPID;

    private ProfiledPIDController armPID;

    private ProfiledPIDController wristPID;

    private Optional<CarriagePreset> carriagePreset = Optional.empty();

    // MAIN CONSTUCTOR
    private ElevatorCarriageSubsystem()
    {
        beamBreakSensor = new DigitalInput(8);

        mLeftElevatorMotor = new TalonFX(kLeftElevatorMotorID, Constants.kCanivoreBusName);

        TalonFXConfiguration leftConfig = new TalonFXConfiguration();
        CurrentLimitsConfigs leftCurrentConfig = new CurrentLimitsConfigs();
        MotorOutputConfigs leftMotorConfig = new MotorOutputConfigs();
        mLeftElevatorMotor.getConfigurator().refresh(leftConfig);
        mLeftElevatorMotor.getConfigurator().refresh(leftCurrentConfig);
        mLeftElevatorMotor.getConfigurator().refresh(leftMotorConfig);
        leftCurrentConfig.SupplyCurrentLimit = 30;
        leftCurrentConfig.SupplyCurrentLimitEnable = true;
        leftCurrentConfig.StatorCurrentLimitEnable = true;
        leftCurrentConfig.StatorCurrentLimit = 30;
        leftMotorConfig.Inverted = InvertedValue.Clockwise_Positive;
        leftMotorConfig.NeutralMode = NeutralModeValue.Brake;
        leftConfig.withCurrentLimits(leftCurrentConfig);
        leftConfig.withMotorOutput(leftMotorConfig);
        mLeftElevatorMotor.getConfigurator().apply(leftConfig);

        mRightElevatorMotor = new TalonFX(kRightElevatorMotorID, Constants.kRioBusName);
        TalonFXConfiguration rightConfig = new TalonFXConfiguration();
        CurrentLimitsConfigs rightCurrentConfig = new CurrentLimitsConfigs();
        MotorOutputConfigs rightMotorConfig = new MotorOutputConfigs();
        mRightElevatorMotor.getConfigurator().refresh(rightConfig);
        mRightElevatorMotor.getConfigurator().refresh(rightCurrentConfig);
        mRightElevatorMotor.getConfigurator().refresh(rightMotorConfig);
        rightCurrentConfig.SupplyCurrentLimit = 30;
        rightCurrentConfig.SupplyCurrentLimitEnable = true;
        rightCurrentConfig.StatorCurrentLimitEnable = true;
        rightCurrentConfig.StatorCurrentLimit = 30;
        rightMotorConfig.Inverted = InvertedValue.Clockwise_Positive;
        rightMotorConfig.NeutralMode = NeutralModeValue.Brake;
        rightConfig.withCurrentLimits(rightCurrentConfig);
        rightConfig.withMotorOutput(rightMotorConfig);
        mRightElevatorMotor.getConfigurator().apply(rightConfig);

        lights = new LightStatusRequest(LightState.kOff, -1);
        LightsSubsystem.instance().addRequest(lights);


        mArmMotor = new TalonFX(kArmMotorId, Constants.kCanivoreBusName);
        TalonFXConfiguration armConfig = new TalonFXConfiguration();
        CurrentLimitsConfigs armCurrentConfig = new CurrentLimitsConfigs();
        MotorOutputConfigs armMotorConfig = new MotorOutputConfigs();
        mArmMotor.getConfigurator().refresh(armConfig);
        mArmMotor.getConfigurator().refresh(armCurrentConfig);
        mArmMotor.getConfigurator().refresh(armMotorConfig);
        armCurrentConfig.SupplyCurrentLimit = 60;
        armCurrentConfig.SupplyCurrentLimitEnable = true;
        armCurrentConfig.StatorCurrentLimitEnable = true;
        armCurrentConfig.StatorCurrentLimit = 60;
        armMotorConfig.NeutralMode = NeutralModeValue.Brake;
        armConfig.withCurrentLimits(armCurrentConfig);
        armConfig.withMotorOutput(armMotorConfig);
        mArmMotor.getConfigurator().apply(armConfig);

        mWristMotor = new TalonFX(kWristMotorId, Constants.kCanivoreBusName);
        TalonFXConfiguration wristConfig = new TalonFXConfiguration();
        CurrentLimitsConfigs wristCurrentConfig = new CurrentLimitsConfigs();
        MotorOutputConfigs wristMotorConfig = new MotorOutputConfigs();
        mWristMotor.getConfigurator().refresh(wristConfig);
        mWristMotor.getConfigurator().refresh(wristCurrentConfig);
        mWristMotor.getConfigurator().refresh(wristMotorConfig);
        wristCurrentConfig.SupplyCurrentLimit = 25;
        wristCurrentConfig.SupplyCurrentLimitEnable = true;
        wristCurrentConfig.StatorCurrentLimitEnable = true;
        wristCurrentConfig.StatorCurrentLimit = 25;
        wristMotorConfig.NeutralMode = NeutralModeValue.Brake;
        wristConfig.withCurrentLimits(wristCurrentConfig);
        wristConfig.withMotorOutput(wristMotorConfig);
        mWristMotor.getConfigurator().apply(wristConfig);

        mArmEncoder = new DutyCycleEncoder(0);
        mWristEncoder = new DutyCycleEncoder(1);

        elevatorPID = new ProfiledPIDController(kElevatorP, kElevatorI, kElevatorD, new Constraints(50, 50));
        armPID = new ProfiledPIDController(kArmP, kArmI, kArmD, new Constraints(720, 720));
        wristPID = new ProfiledPIDController(kWristP, kWristI, kWristD, new Constraints(360, 360));
    }

    public boolean isAtTheBottom()
    {
        return !beamBreakSensor.get();
    }

    // STOP EVERYTHING METHOD
    public void stop()
    {
        carriagePreset = Optional.empty();
        mLeftElevatorMotor.stopMotor();
        mRightElevatorMotor.stopMotor();
        mArmMotor.stopMotor();
        mWristMotor.stopMotor();
    }

    public void resetElevatorEncoder()
    {
        mLeftElevatorMotor.setPosition(0);
        mRightElevatorMotor.setPosition(0);
        elevatorPID.reset(initialHeight);
    }

    // SET ELEVATOR VOLTAGE METHOD
    private void setElevatorVoltage(double voltage)
    {
        mLeftElevatorMotor.setVoltage(voltage + kElevatorKG);
        mRightElevatorMotor.setVoltage(voltage + kElevatorKG);
    }

    public void setManualElevatorVoltage(double voltage)
    {
        carriagePreset = Optional.empty();
        setElevatorVoltage(voltage);
    }

    // SET CARRIAGE VOLTAGE METHOD
    private void setArmVoltage(double voltage) 
    {
        mArmMotor.setVoltage(voltage + kArmKG * Math.cos(Math.toRadians(this.getArmAngle())));
    }

    public void setManualArmVoltage(double voltage) 
    {
        carriagePreset = Optional.empty();
        setArmVoltage(voltage);
    }


    private void setWristVoltage(double voltage) 
    {
        mWristMotor.setVoltage(voltage + kWristKG * Math.cos(Math.toRadians(this.getAbsoluteWristAngle())));
    }

    public void setManualWristVoltage(double voltage) 
    {
        carriagePreset = Optional.empty();
        setWristVoltage(voltage);
    }

    // SET PRESET METHOD
    public void setPreset(CarriagePreset preset)
    {
        carriagePreset = Optional.of(preset);
    }

    public Optional<CarriagePreset> getActivePreset()
    {
        return carriagePreset;
    }

    // GET ELEVATOR POSITION METHOD
    public double getElevatorPosition()
    {
        return mLeftElevatorMotor.getPosition().getValueAsDouble() * rotationsToInches + initialHeight;
    }

    // GET CARRIAGE X POSITION METHOD
    public double getCarriageX()
    {
        double armX = 13 * Math.cos(Math.toRadians(getArmAngle()));

        double wristX = -10 * Math.cos(Math.toRadians(getAbsoluteWristAngle())) + -6 * Math.sin(Math.toRadians(getAbsoluteWristAngle()));

        return armX + wristX;
    }

    // GET CARRIAGE Y POSITION METHOD
    public double getCarriageYPosition()
    {
        return getElevatorHeight();
        // NEEDS MATH
    }

    // CHECKS IF ITS AT BOTTOM METHOD
    public boolean isAtBottom()
    {
        return bottomSwitch.get();
    }

    // CHECKS IF ITS AT TOP METHOD
    public boolean isAtTop()
    {
        return topSwitch.get();
    }

    // GETS ARM ANGLE METHOD
    public double getArmAngle() 
    {
        double armAngle = mArmEncoder.get() * 360;

        return armAngle - armOffset;
    }

    public double getWristAngle()
    {
        double wristAngle = mWristEncoder.get() * 360;
        
        // Compensate for values above our expected range.
        // If the true angle is above our range, subtract a full rotation.
        if (wristAngle > angleOverflowMin) wristAngle -= 360;

        return wristAngle - wristOffset;
    }

    // GETS WRIST ANGLE METHOD
    public double getAbsoluteWristAngle() 
    {
        return mWristEncoder.get() * 360;
    }

    // RESETS PID METHOD
    public void resetPID()
    {
        elevatorPID.reset(getElevatorHeight());
        armPID.reset(getArmAngle());
        wristPID.reset(getWristAngle());
    }

    // MOVES ELEVATOR METHOD
    private void moveElevator(CarriagePreset preset)
    {
        double elevatorOutput = elevatorPID.calculate(getElevatorHeight(), preset.kElevatorPreset) + kElevatorKG;

        mLeftElevatorMotor.set(elevatorOutput);

        mRightElevatorMotor.set(elevatorOutput);
    }

    // MOVES ARM METHOD
    private void moveArm(CarriagePreset preset)
    {
        double armOutput = armPID.calculate(getArmAngle(), preset.kArmPreset);

        setArmVoltage(armOutput);
    }

    // MOVES WRIST METHOD
    private void moveWrist(CarriagePreset preset)
    {
        double wristOutput = wristPID.calculate(getWristAngle(), preset.kWristPreset);

        setWristVoltage(wristOutput);
    }

    // GETS ELEVATOR HEIGHT METHOD
    public double getElevatorHeight()
    {
        return mLeftElevatorMotor.getPosition().getValueAsDouble() * rotationsToInches + initialHeight;
    }

    // NEEDED TO CHECK IF POSITION IS THERE
    public boolean isAtPosition(CarriagePreset preset)
    {
        double curHeight = getElevatorHeight();
        double tarHeight = preset.kElevatorPreset;
        double needed = 0.0; // THIS NUMBER NEEDS TO BE UPDATED
        return (curHeight - tarHeight) <= needed;
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

    // MAIN PERIODIC METHOD
    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Elevator Position", getElevatorPosition());
        SmartDashboard.putNumber("Arm Position", getArmAngle());
        SmartDashboard.putNumber("Wrist Position", getWristAngle());

        if (carriagePreset.isPresent())
        {
            CarriagePreset preset = carriagePreset.get();

            moveElevator(preset);

            moveArm(preset);

            moveWrist(preset);
        }

        periodicLights();
    }

    // LIGHT PERIODIC METHOD
    private void periodicLights()
    {
        final double kLightSpeedTolerance = 0.1;

        if (Robot.sIsTeleop()) lights.priority = 201;

        else lights.priority = 101;

        double speed = mLeftElevatorMotor.get();

        if (speed > kLightSpeedTolerance)
        {
            lights.active = true;

            if (Robot.sIsTeleop())
            {
                lights.priority = 201;

                lights.state = LightState.kTeleopElevatorUp;
            }

            else
            {
                lights.priority = 101;

                lights.state = LightState.kAutonomousElevatorUp;
            }
        }

        else if (speed < -kLightSpeedTolerance)
        {
            lights.active = true;

            if (Robot.sIsTeleop()) 
            {
                lights.state = LightState.kTeleopElevatorDown;
            }

            else 
            {
                lights.state = LightState.kAutonomousElevatorDown;
            }
        }

        else 
        {
            lights.active = false;
        }
    }
}
