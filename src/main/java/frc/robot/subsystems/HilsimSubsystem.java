package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HilsimSubsystem extends SubsystemBase
{
    private final SparkMax mMotor1;

    public HilsimSubsystem()
    {
        mMotor1 = new SparkMax(9, MotorType.kBrushless);
    }

    @Override
    public void periodic()
    {
        mMotor1.set(1);
    }

    @Override
    public void simulationPeriodic()
    {
        
    }
}
