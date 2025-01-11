package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSubsystem extends SubsystemBase
{
    private final SparkMax mWristMotor;
    private boolean WristMotorOn = true;

    public WristSubsystem()
    {
        mWristMotor = new SparkMax(0, MotorType.kBrushless);
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
    }
    @Override
    public void simulationPeriodic() {
        // TODO Auto-generated method stub
        super.simulationPeriodic();
    }

    public void setMotorOff()
    {
        mWristMotor.setVoltage(0);
        WristMotorOn = false;
    }

    public void setMotorVolts(double voltage)
    {
        mWristMotor.setVoltage(voltage);
        WristMotorOn = true;
    }
}
