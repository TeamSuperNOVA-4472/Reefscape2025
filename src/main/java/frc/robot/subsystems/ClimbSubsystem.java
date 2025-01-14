package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

// TODO: Actual motor ports
public class ClimbSubsystem extends SubsystemBase
{
    private final PWMTalonFX mLeftClimbMotor;
    private final PWMTalonFX mRightClimbMotor;

    private double mCurrentVoltage = 0; // Starts at 0 voltage!

    // Constructor
    public ClimbSubsystem() // Last year, checked if robot was real for testing - not required, in fact is bad?
    {
        mLeftClimbMotor = new PWMTalonFX(1);
        mRightClimbMotor = new PWMTalonFX(0); // Two krakens using TalonFX motor controllers
        // I think this is all that I need, do we need anything other than bang-bang control?
    }

    public void setVoltage(double pNewVoltage)
    {
        mCurrentVoltage = pNewVoltage;

        if(Robot.isReal())
        {
            updateVoltage();
        }
    }

    // Private update method. idk if this is required but I think it's good practice - and makes life easier!
    private void updateVoltage()
    {
        mLeftClimbMotor.setVoltage(mCurrentVoltage);
        mRightClimbMotor.setVoltage(mCurrentVoltage);
    }

    // Gets the voltage of the climb subsystem.
    public double getVoltage()
    {
        return mCurrentVoltage;
    }

    // Run constantly to ensure that voltage is what it should be
    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Climb Voltage", mCurrentVoltage);
    }
}
