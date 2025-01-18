package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase
{
    public static final int kElevatorMotorId = 1;

    public static final int kBottomSwitchChannel = 1;
    public static final int kTopSwitchChannel = 1;

    private boolean isMovingUp = false;
    private boolean isMovingDown = false;

    private SparkMax elevatorMotor;

    private DigitalInput bottomSwitch;
    private DigitalInput topSwitch;

    public ElevatorSubsystem()
    {
        bottomSwitch = new DigitalInput(kBottomSwitchChannel);
        topSwitch = new DigitalInput(kTopSwitchChannel);

        elevatorMotor = new SparkMax(kElevatorMotorId, MotorType.kBrushless);
    }

    public void stop()
    {
        elevatorMotor.stopMotor();
        isMovingUp = false;
        isMovingDown = false;
    }

    public void setVoltage(double voltage)
    {
        elevatorMotor.setVoltage(voltage);

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
}
