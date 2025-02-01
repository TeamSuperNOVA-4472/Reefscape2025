package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

// TODO: Actual motor ports
public class ClimbSubsystem extends SubsystemBase
{

    // THE CONSTANTS
    private static final int LEFT_MOTOR = 0;
    private static final int RIGHT_MOTOR = 0;
    private static final int GRABBER_MOTOR = 0;

    private final PWMTalonFX mLeftClimbMotor;
    private final PWMTalonFX mRightClimbMotor;
    private final PWMTalonFX mGrabberMotor;

    private double mClimbVoltage = 0;
    private double mGrabberVoltage = 0;

    // Constructor
    public ClimbSubsystem()
    {
        mLeftClimbMotor = new PWMTalonFX(LEFT_MOTOR); // Move these to constants at the top, DONE
        mRightClimbMotor = new PWMTalonFX(RIGHT_MOTOR); // Two krakens using TalonFX motor controllers
        mGrabberMotor = new PWMTalonFX(GRABBER_MOTOR);  // Two krakens using TalonFX motor controllers
    }

    public void setClimbVoltage(double pNewVoltage)
    {
        mClimbVoltage = pNewVoltage;

        if(Robot.isReal())
        {
            updateClimbVoltage();
        }
    }

    public void setGrabberVoltage(double pNewVoltage)
    {
        mGrabberVoltage = pNewVoltage;

        if (Robot.isReal())
        {
            updateGrabberVoltage();
        }
    }

    // Private update method. idk if this is required but I think it's good practice - and makes life easier!
    private void updateClimbVoltage()
    {
        mLeftClimbMotor.setVoltage(mClimbVoltage);
        mRightClimbMotor.setVoltage(mClimbVoltage);
    }

    private void updateGrabberVoltage()
    {
        mGrabberMotor.setVoltage(mGrabberVoltage);
    }

    // Gets the voltage of the climb subsystem.
    public double getClimbVoltage()
    {
        return mClimbVoltage;
    }

    public double getGrabberVoltage()
    {
        return mGrabberVoltage;
    }

    // Run constantly to ensure that voltage is what it should be
    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Climb Voltage", mClimbVoltage);
        SmartDashboard.putNumber("Grabber Voltage", mGrabberVoltage);
    }
}
