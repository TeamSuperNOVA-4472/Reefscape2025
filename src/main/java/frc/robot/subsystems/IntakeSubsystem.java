package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase 
{
    private final SparkMax mIntakeMotor;
    private boolean IntakeMotorOn;

    public IntakeSubsystem()
    {
        mIntakeMotor = new SparkMax(0, MotorType.kBrushless);
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

    public void setIntakeOff()
    {
        mIntakeMotor.setVoltage(0);
        IntakeMotorOn = false;
    }

    public void setIntakeVolts(double voltage)
    {
        mIntakeMotor.setVoltage(voltage);
        IntakeMotorOn = true;
    }

}
