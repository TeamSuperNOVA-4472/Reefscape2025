package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.objectmodels.IntakePresets;

public class ElevatorSubsystem extends SubsystemBase
{
    public static final int kElevatorMotorID = 1;

    public static final int kBottomSwitchChannel = 1;
    public static final int kTopSwitchChannel = 1;

    public static final double kPresetAway = 0;
    public static final double kPresetGroundPickup = 0;
    public static final double kPresetL1 = 1;
    public static final double kPresetL2 = 2;
    public static final double kPresetL3 = 3;
    public static final double kPresetL4 = 4;

    public static final double kElevatorP = 1;
    public static final double kElevatorI = 1;
    public static final double kElevatorD = 1;

    private Optional<IntakePresets> activePreset = Optional.empty();

    private PIDController elevatorPID;

    private boolean isMovingUp = false;
    private boolean isMovingDown = false;

    private TalonFX elevatorMotor;

    private DigitalInput bottomSwitch;
    private DigitalInput topSwitch;

    public ElevatorSubsystem()
    {
        bottomSwitch = new DigitalInput(kBottomSwitchChannel);
        topSwitch = new DigitalInput(kTopSwitchChannel);

        elevatorMotor = new TalonFX(kElevatorMotorID);
        elevatorPID = new PIDController(kElevatorP, kElevatorI, kElevatorD);
    }

    public void stop()
    {
        elevatorMotor.stopMotor();
        isMovingUp = false;
        isMovingDown = false;
        activePreset = Optional.empty();
    }

    public void setVoltage(double voltage)
    {
        final double kVoltageTolerance = 0.1;

        elevatorMotor.setVoltage(voltage);
        activePreset = Optional.empty();

        if (voltage > kVoltageTolerance)
        {
            isMovingUp = true;
            isMovingDown = false;
        }

        else if (voltage < -kVoltageTolerance)
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

        double currentPosition = elevatorMotor.getPosition().getValueAsDouble();
        double newSpeed = elevatorPID.calculate(currentPosition);
        elevatorMotor.set(newSpeed);
    }
}
