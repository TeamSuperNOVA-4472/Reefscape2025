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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.objectmodels.LightState;
import frc.robot.objectmodels.LightStatusRequest;

public class ElevatorSubsystem extends SubsystemBase
{
    public static final int kLeftElevatorMotorID = 0;
    public static final int kRightElevatorMotorID = 1;

    public static final int kBottomSwitchChannel = 0;
    public static final int kTopSwitchChannel = 1;

    
    public static final double initialHeight = 12.875;
    public static final double kPresetAway = initialHeight;
    public static final double kPresetGroundPickup = initialHeight;
    public static final double kPresetCoralPickup = 19.522144;
    public static final double kPresetL1 = 12.911;
    public static final double kPresetL2 = 21.691;
    public static final double kPresetL3 = 38.5;
    public static final double kPresetL4 = 65.5;

    public static final double kPresetBarge = 65;
    public static final double kPresetProcessor = initialHeight;
    public static final double kPresetGround = 25;
    public static final double kPresetAlgaeL2 = 25;
    public static final double kPresetAlgaeL3 = 43.344;

    public static final double kElevatorP = 0.9;
    public static final double kElevatorI = 0;
    public static final double kElevatorD = 0;
    public static final double kG = 0.2;

    public static final double rotationsToInches = 0.45;

    private final TalonFX mElevatorLeft;
    //private final TalonFX mElevatorRight;

    private Optional<Double> activePreset = Optional.empty();

    private ProfiledPIDController elevatorPID;
    private boolean isMovingUp = false;
    private boolean isMovingDown = false;


    private DigitalInput bottomSwitch;
    private DigitalInput topSwitch;

    private LightStatusRequest lights;

    public ElevatorSubsystem(LightsSubsystem lightController)
    {
        //motorConfigRight = new MotorOutputConfigs();
        mElevatorLeft = new TalonFX(kLeftElevatorMotorID, Constants.kCanivoreBusName);
        TalonFXConfiguration leftConfig = new TalonFXConfiguration();
        CurrentLimitsConfigs leftCurrentConfig = new CurrentLimitsConfigs();
        MotorOutputConfigs leftMotorConfig = new MotorOutputConfigs();
        mElevatorLeft.getConfigurator().refresh(leftConfig);
        mElevatorLeft.getConfigurator().refresh(leftCurrentConfig);
        mElevatorLeft.getConfigurator().refresh(leftMotorConfig);
        leftCurrentConfig.SupplyCurrentLimit = 30;
        leftCurrentConfig.SupplyCurrentLimitEnable = true;
        leftCurrentConfig.StatorCurrentLimitEnable = true;
        leftCurrentConfig.StatorCurrentLimit = 30;
        leftMotorConfig.Inverted = InvertedValue.Clockwise_Positive;
        leftMotorConfig.NeutralMode = NeutralModeValue.Brake;
        leftConfig.withCurrentLimits(leftCurrentConfig);
        leftConfig.withMotorOutput(leftMotorConfig);
        mElevatorLeft.getConfigurator().apply(leftConfig);
        lights = new LightStatusRequest(LightState.kOff, -1);
        lightController.addRequest(lights);
        elevatorPID = new ProfiledPIDController(kElevatorP, kElevatorI, kElevatorD, new Constraints(50, 50));
        activePreset = Optional.of(initialHeight);
    }

    public void stop()
    {
        mElevatorLeft.stopMotor();
        isMovingUp = false;
        isMovingDown = false;
        activePreset = Optional.empty();
    }

    private void setVoltage(double voltage)
    {
        final double kVoltageTolerance = 0.1;

        
        mElevatorLeft.setVoltage(voltage + kG);

        if (voltage > kVoltageTolerance + kG)
        {
            isMovingUp = true;
            isMovingDown = false;
        }

        else if (voltage < -kVoltageTolerance + kG)
        {
            isMovingDown = true;
            isMovingUp = false;
        }

        else
        {
            isMovingUp = false; 
            isMovingDown = false;
        }
    }

    public void setManualVoltage(double voltage)
    {
        activePreset = Optional.empty();
        setVoltage(voltage);
    }

    public boolean isAtBottom()
    {
        return bottomSwitch.get();
    }
    public boolean isAtTop()
    {
        return topSwitch.get();
    }
    public boolean isMoving()
    {
        return isMovingUp || isMovingDown;
    }

    public void setPreset(Double preset)
    {
        activePreset = Optional.of(preset);
    }

    public void resetEncoder(){
        mElevatorLeft.setPosition(0);
        elevatorPID.reset(initialHeight);
    }

    public double getElevatorHeight(){
        return mElevatorLeft.getPosition().getValueAsDouble()*rotationsToInches + initialHeight;
    }

    public double getElevatorPreset(){
        return activePreset.orElse(0.0);
    }

    public void resetElevatorPID(){
        elevatorPID.reset(this.getElevatorHeight());
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Left Elevator Output", mElevatorLeft.getPosition().getValueAsDouble());
        SmartDashboard.putString("Elevator Units", mElevatorLeft.getPosition().getUnits());
        SmartDashboard.putNumber("Elevator Height", this.getElevatorHeight());

        periodicLights();

        if (activePreset.isEmpty()) return; // No preset.

        double currentPosition = this.getElevatorHeight();
        double newSpeed = elevatorPID.calculate(currentPosition, activePreset.get());
        this.setVoltage(newSpeed);

        SmartDashboard.putNumber("Elevator Speed", newSpeed);
    }

    private void periodicLights()
    {
        // Lights
        final double kLightSpeedTolerance = 0.1;
        if (Robot.sIsTeleop()) lights.priority = 201;
        else lights.priority = 101;

        double speed = mElevatorLeft.get();
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
            if (Robot.sIsTeleop()) lights.state = LightState.kTeleopElevatorDown;
            else lights.state = LightState.kAutonomousElevatorDown;
        }
        else lights.active = false;
    }
}
