package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase 
{

    public static final int kIntakeMotorId = 1;
    public static final double kIntakeVoltage = 12;

    private boolean mIsIntaking;
    private boolean mIsOutTaking;
    private SparkMax mIntakeMotor;

    public IntakeSubsystem()
    {
        mIntakeMotor = new SparkMax(kIntakeMotorId, MotorType.kBrushless);
    }

    public void inTake()
    {
        mIntakeMotor.setVoltage(kIntakeVoltage);
        mIsIntaking = true;
        mIsOutTaking = false;

    }

    public void outTake()
    {
        mIntakeMotor.setVoltage(-kIntakeVoltage);
        mIsIntaking = false;
        mIsOutTaking = true;
    }

    public void stop()
    {
        mIntakeMotor.stopMotor();
        mIsIntaking = false;
        mIsOutTaking = false;
    }

    public boolean isIntaking()
    {
        return mIsIntaking;
    }

    public boolean isOutTaking()
    {
        return mIsOutTaking;
    }

    @Override
    public void periodic() 
    {
        // TODO Auto-generated method stub
        super.periodic();
    }
    @Override
    public void simulationPeriodic() 
    {
        // TODO Auto-generated method stub
        super.simulationPeriodic();
    }

}
