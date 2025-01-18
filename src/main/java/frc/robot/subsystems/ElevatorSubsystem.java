package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase
{
    public static final int kElevatorMotorID = 1;

    public static final int kBottomSwitchChannel = 1;
    public static final int kTopSwitchChannel = 1;

    public static final double kPresetBottom = 0;
    public static final double kPresetL1 = 1;
    public static final double kPresetL2 = 2;
    public static final double kPresetL3 = 3;
    public static final double kPresetL4 = 4;

    public static final double kElevatorP = 1;
    public static final double kElevatorI = 1;
    public static final double kElevatorD = 1;

    private int activePreset = -1;

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
        activePreset = -1;
    }

    public void setVoltage(double voltage)
    {
        elevatorMotor.setVoltage(voltage);

        activePreset = -1;

        if (voltage > 0)
        {
            isMovingUp = true;
            isMovingDown = false;
        }

        else if (voltage < 0)
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

    public void setPreset(int presetNum)
    {
        activePreset = presetNum;
    }

    @Override
    public void periodic()
    {

        double desiredPosition;

        if (activePreset == 0)
        {
            desiredPosition = kPresetBottom;
        }

        else if (activePreset == 1)
        {
            desiredPosition = kPresetL1;
        }

        else if (activePreset == 2)
        {
            desiredPosition = kPresetL2;
        }

        else if (activePreset == 3)
        {
            desiredPosition = kPresetL3;
        }

        else if (activePreset == 4)
        {
            desiredPosition = kPresetL4;
        }

        else
        {
            return;
        }

        elevatorPID.setSetpoint(desiredPosition);

        double currentPosition = elevatorMotor.getPosition().getValueAsDouble();

        double newSpeed = elevatorPID.calculate(currentPosition);
        elevatorMotor.set(newSpeed);

    }
}
