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
import frc.robot.Robot;
import frc.robot.objectmodels.CarriagePreset;
import frc.robot.objectmodels.LightState;
import frc.robot.objectmodels.LightStatusRequest;

// ALL MAIN VARIABLES
public class ElevatorCarriageSubsystem extends SubsystemBase
{
    public static final int kLeftElevatorMotorID = 0;

    public static final int kRightElevatorMotorID = 1;

    public static final int kArmMotorId = 22;

    public static final int kWristMotorId = 20;

    private final DutyCycleEncoder mElevatorEncoder;

    private final DutyCycleEncoder mWristEncoder;

    private final DutyCycleEncoder mArmEncoder;

    private DigitalInput bottomSwitch;

    private DigitalInput topSwitch;

    private LightStatusRequest lights;

    public static final double initialHeight = 12.875;

    private static final double kElevatorP = 0.9;

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

    private static final double rotationsToInches = 0.45;

    private final TalonFX mLeftElevatorMotor;

    private final TalonFX mRightElevatorMotor;

    private final TalonFX mArmMotor;

    private final TalonFX mWristMotor;

    private ProfiledPIDController elevatorPID;

    private ProfiledPIDController armPID;

    private ProfiledPIDController wristPID;

    private Optional<CarriagePreset> carriagePreset = Optional.empty();

    // MAIN CONSTUCTOR
    public ElevatorCarriageSubsystem()
    {
        mLeftElevatorMotor = new TalonFX(kLeftElevatorMotorID, "CANivore");

        mRightElevatorMotor = new TalonFX(kRightElevatorMotorID, "CANivore");

        mArmMotor = new TalonFX(kArmMotorId, "CANivore");

        mWristMotor = new TalonFX(kWristMotorId, "CANivore");

        TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();

        CurrentLimitsConfigs elevatorCurrentConfig = new CurrentLimitsConfigs();

        MotorOutputConfigs elevatorMotorConfig = new MotorOutputConfigs();

        mLeftElevatorMotor.getConfigurator().refresh(elevatorConfig);

        mRightElevatorMotor.getConfigurator().refresh(elevatorConfig);

        elevatorCurrentConfig.SupplyCurrentLimit = 30;

        elevatorCurrentConfig.SupplyCurrentLimitEnable = true;

        elevatorMotorConfig.Inverted = InvertedValue.Clockwise_Positive;

        elevatorMotorConfig.NeutralMode = NeutralModeValue.Brake;

        elevatorConfig.withCurrentLimits(elevatorCurrentConfig);

        elevatorConfig.withMotorOutput(elevatorMotorConfig);

        mLeftElevatorMotor.getConfigurator().apply(elevatorConfig);

        mRightElevatorMotor.getConfigurator().apply(elevatorConfig);

        mElevatorEncoder = new DutyCycleEncoder(0);

        mArmEncoder = new DutyCycleEncoder(1);

        mWristEncoder = new DutyCycleEncoder(2);

        elevatorPID = new ProfiledPIDController(kElevatorP, kElevatorI, kElevatorD, new Constraints(50, 50));

        armPID = new ProfiledPIDController(kArmP, kArmI, kArmD, new Constraints(720, 720));

        wristPID = new ProfiledPIDController(kWristP, kWristI, kWristD, new Constraints(360, 360));
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

    // SET ELEVATOR VOLTAGE METHOD
    public void setElevatorVoltage(double voltage)
    {
        mLeftElevatorMotor.setVoltage(voltage);

        mRightElevatorMotor.setVoltage(voltage);
    }

    // SET CARRIAGE VOLTAGE METHOD
    public void setCarriageVoltage(double voltage)
    {
        mArmMotor.setVoltage(voltage);

        mWristMotor.setVoltage(voltage);
    }

    // SET PRESET METHOD
    public void setPreset(CarriagePreset preset)
    {
        carriagePreset = Optional.of(preset);
    }

    // GET ELEVATOR POSITION METHOD
    public double getElevatorPosition()
    {
        return mElevatorEncoder.get() * 360;
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
        return mArmEncoder.get() * 360;
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

        wristPID.reset(getAbsoluteWristAngle());
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
        double armOutput = armPID.calculate(getArmAngle(), preset.kArmPreset) + kArmKG;

        mArmMotor.set(armOutput);
    }

    // MOVES WRIST METHOD
    private void moveWrist(CarriagePreset preset)
    {
        double wristOutput = wristPID.calculate(getAbsoluteWristAngle(), preset.kWristPreset) + kWristKG;

        mWristMotor.set(wristOutput);
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

    // MAIN PERIODIC METHOD
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
