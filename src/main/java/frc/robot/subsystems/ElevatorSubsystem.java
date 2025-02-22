package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.objectmodels.IntakePresets;
import frc.robot.objectmodels.LightState;
import frc.robot.objectmodels.LightStatusRequest;

public class ElevatorSubsystem extends SubsystemBase
{
    public static final int kLeftElevatorMotorID = 0;
    public static final int kRightElevatorMotorID = 1;

    public static final int kBottomSwitchChannel = 0;
    public static final int kTopSwitchChannel = 1;

    public static final double kPresetAway = 0;
    public static final double kPresetGroundPickup = 0;
    public static final double kPresetL1 = 1;
    public static final double kPresetL2 = 2;
    public static final double kPresetL3 = 3;
    public static final double kPresetL4 = 4;

    public static final double kElevatorP = 0;
    public static final double kElevatorI = 0;
    public static final double kElevatorD = 0;
    public static final double kG = 0.2;

    private final TalonFX mElevatorLeft;
    //private final TalonFX mElevatorRight;

    private Optional<IntakePresets> activePreset = Optional.empty();

    private PIDController elevatorPID;
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
        leftCurrentConfig.SupplyCurrentLimit = 60;
        leftCurrentConfig.SupplyCurrentLimitEnable = true;
        leftCurrentConfig.StatorCurrentLimitEnable = true;
        leftCurrentConfig.StatorCurrentLimit = 60;
        leftMotorConfig.Inverted = InvertedValue.Clockwise_Positive;
        leftMotorConfig.NeutralMode = NeutralModeValue.Brake;
        leftConfig.withCurrentLimits(leftCurrentConfig);
        leftConfig.withMotorOutput(leftMotorConfig);
        mElevatorLeft.getConfigurator().apply(leftConfig);
        elevatorPID = new PIDController(kElevatorP, kElevatorI, kElevatorD);

        lights = new LightStatusRequest(LightState.kTeleopElevatorUp, 201);
    }

    public void stop()
    {
        mElevatorLeft.stopMotor();
        isMovingUp = false;
        isMovingDown = false;
        activePreset = Optional.empty();
    }

    public void setVoltage(double voltage)
    {
        final double kVoltageTolerance = 0.1;

        
        mElevatorLeft.setVoltage(voltage + kG);
        //mElevatorRight.setVoltage(voltage);
        activePreset = Optional.empty();

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

    public void setPreset(IntakePresets preset)
    {
        activePreset = Optional.of(preset);
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Left Elevator Output", mElevatorLeft.getPosition().getValueAsDouble());
        SmartDashboard.putString("Elevator Units", mElevatorLeft.getPosition().getUnits());
        //SmartDashboard.putNumber("Right Elevator Output", mElevatorRight.getPosition().getValueAsDouble());


        if (activePreset.isEmpty()) return; // No preset.
        else
        {
            switch (activePreset.get())
            {
                case kAway:
                    elevatorPID.setSetpoint(kPresetAway);
                    break;
                
                case kGroundPickup:
                    elevatorPID.setSetpoint(kPresetGroundPickup);
                    break;

                case kScoreL1:
                    elevatorPID.setSetpoint(kPresetL1);
                    break;
                    
                case kScoreL2:
                    elevatorPID.setSetpoint(kPresetL2);
                    break;
                
                case kScoreL3:
                    elevatorPID.setSetpoint(kPresetL3);
                    break;
                    
                case kScoreL4:
                    elevatorPID.setSetpoint(kPresetL4);
                    break;
                
                default: return; // Unknown preset. Shouldn't happen.
            }
        }

        double currentPosition = mElevatorLeft.getPosition().getValueAsDouble();
        double newSpeed = elevatorPID.calculate(currentPosition);
        mElevatorLeft.set(newSpeed);

        // Lights
    }
}
