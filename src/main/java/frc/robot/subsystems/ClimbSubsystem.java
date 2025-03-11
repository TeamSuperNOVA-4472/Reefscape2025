package frc.robot.subsystems;

import static frc.robot.subsystems.SwerveSubsystem.kA;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

// TODO: Actual motor ports
public class ClimbSubsystem extends SubsystemBase
{
    private static ClimbSubsystem kInstance;
    public static ClimbSubsystem instance()
    {
        return kInstance;
    }

    // THE CONSTANTS
    private static final int LEFT_MOTOR = 12;
    private static final int RIGHT_MOTOR = 13;
    private static final int GRABBER_MOTOR = 35;
    private static final int LEFT_ENCODER = 3; //either 0, 1, 2, or 3 for both encoders
    private static final int RIGHT_ENCODER = 2;
    private static final double RIGHT_CLIMB_OUT_ANGLE = 0;
    private static final double LEFT_CLIMB_OUT_ANGLE = 0;
    private static final double RIGHT_CLIMB_IN_ANGLE = 0;
    private static final double LEFT_CLIMB_IN_ANGLE = 0;
    private static final int LEFT_CLIMB_P = 0;
    private static final int LEFT_CLIMB_I = 0;
    private static final int LEFT_CLIMB_D = 0;
    private static final int RIGHT_CLIMB_P = 0;
    private static final int RIGHT_CLIMB_I = 0;
    private static final int RIGHT_CLIMB_D = 0;

    private static final double LEFT_CLIMB_OFFSET = -135;

    private static final double RIGHT_CLIMB_OFFSET = 150;
    private static final double MAX_LEFT_CLIMB = 90;
    private static final double MAX_RIGHT_CLIMB = 190;
    private static final double MIN_LEFT_CLIMB = -10;
    private static final double MIN_RIGHT_CLIMB = -17;
    
    private final TalonFX mLeftClimbMotor;
    private final TalonFX mRightClimbMotor;
    private final SparkMax mGrabberMotor;

    private final PIDController mLeftClimbPID;
    private final PIDController mRightClimbPID;

    private final DutyCycleEncoder mLeftClimbEncoder;
    private final DutyCycleEncoder mRightClimbEncoder;

    private double mLeftClimbVoltage = 0;
    private double mRightClimbVoltage = 0;
    private double mGrabberVoltage = 0;

    private boolean mClosing; // Don't have time for a better implementation.

    private double mLeftTargetAngle = LEFT_CLIMB_OUT_ANGLE;
    private double mRightTargetAngle = RIGHT_CLIMB_OUT_ANGLE;
    // Constructor
    public ClimbSubsystem()
    {
        kInstance = this;

        mLeftClimbMotor = new TalonFX(LEFT_MOTOR, "CANivore"); // Move these to constants at the top, DONE
        mRightClimbMotor = new TalonFX(RIGHT_MOTOR); // Two krakens using TalonFX motor controllers
        mGrabberMotor = new SparkMax(GRABBER_MOTOR, MotorType.kBrushless);  // Two krakens using TalonFX motor controllers

        mLeftClimbEncoder = new DutyCycleEncoder(LEFT_ENCODER);
        mRightClimbEncoder = new DutyCycleEncoder(RIGHT_ENCODER);

        mLeftClimbPID = new PIDController(LEFT_CLIMB_P, LEFT_CLIMB_I, LEFT_CLIMB_D);
        mRightClimbPID = new PIDController(RIGHT_CLIMB_P, RIGHT_CLIMB_I, RIGHT_CLIMB_D);

        mClosing = false;
    }

    public void setGrabberVoltage(double pNewVoltage)
    {
        mGrabberVoltage = pNewVoltage;

        if (Robot.isReal())
        {
            updateGrabberVoltage();
        }
    }

    private void updateGrabberVoltage()
    {
        mGrabberMotor.setVoltage(mGrabberVoltage);
    }

    // Gets the voltage of the climb subsystem.
    public double getLeftClimbVoltage()
    {
        return mLeftClimbVoltage;
    }
    
    public double getRightClimbVoltage()
    {
        return mRightClimbVoltage;
    }

    public void setLeftClimbVoltage(double pNewVoltage)
    {
        mLeftClimbVoltage = pNewVoltage;
    }
    
    public void setRightClimbVoltage(double pNewVoltage)
    {
        mRightClimbVoltage = pNewVoltage; 
    }

    public double getGrabberVoltage()
    {
        return mGrabberVoltage;
    }

    public void openClimber()
    {
        mLeftTargetAngle = LEFT_CLIMB_OUT_ANGLE;
        mRightTargetAngle = RIGHT_CLIMB_OUT_ANGLE;
        mClosing = false;
    }

    public void closeClimber()
    {
        mLeftTargetAngle = LEFT_CLIMB_IN_ANGLE;
        mRightTargetAngle = RIGHT_CLIMB_IN_ANGLE;
        mClosing = true;
    }

    public boolean isClosingOrClosed()
    {
        return mClosing;
    }

    public double getLeftClimbAngle()
    {
        return mLeftClimbEncoder.get()*360 + LEFT_CLIMB_OFFSET;
    }

    public double getRightClimbAngle()
    {

        double value = mRightClimbEncoder.get() * 360 + RIGHT_CLIMB_OFFSET;
        if (value >= 250) value -= 360;
        return value;
    }
    // Run constantly to ensure that voltage is what it should be
    @Override
    public void periodic()
    {
        //mLeftClimbVoltage = mLeftClimbPID.calculate(getLeftClimbAngle(), mLeftTargetAngle);
        //mRightClimbVoltage = mLeftClimbPID.calculate(getRightClimbAngle(), mRightTargetAngle);
        if((getLeftClimbAngle() < MAX_LEFT_CLIMB && mLeftClimbVoltage > 0) 
            || (getLeftClimbAngle() > MIN_LEFT_CLIMB && mLeftClimbVoltage < 0 && getRightClimbAngle() <= getLeftClimbAngle())) {
            mLeftClimbMotor.setVoltage(mLeftClimbVoltage);
        } else {
            mLeftClimbMotor.setVoltage(0.0);
        }

        if((getRightClimbAngle() <  MAX_RIGHT_CLIMB && mRightClimbVoltage > 0) 
            || (getRightClimbAngle() > MIN_RIGHT_CLIMB && mRightClimbVoltage < 0)) {
            mRightClimbMotor.setVoltage(mRightClimbVoltage);
        } else {
            mRightClimbMotor.setVoltage(0.0);
        }
        
        SmartDashboard.putNumber("Left Climb Voltage", mLeftClimbVoltage);
        SmartDashboard.putNumber("Right Climb Voltage", mRightClimbVoltage);
        SmartDashboard.putNumber("Grabber Voltage", mGrabberVoltage);
        SmartDashboard.putNumber("Left Climb angle", getLeftClimbAngle());
        SmartDashboard.putNumber("Right Climb angle", getRightClimbAngle());

        
    }
}
