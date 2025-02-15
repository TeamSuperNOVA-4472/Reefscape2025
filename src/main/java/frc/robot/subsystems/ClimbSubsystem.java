package frc.robot.subsystems;

import static frc.robot.subsystems.SwerveSubsystem.kA;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

// TODO: Actual motor ports
public class ClimbSubsystem extends SubsystemBase
{

    // THE CONSTANTS
    private static final int LEFT_MOTOR = 8;
    private static final int RIGHT_MOTOR = 7;
    private static final int GRABBER_MOTOR = 6;
    private static final int LEFT_ENCODER = 5;
    private static final int RIGHT_ENCODER = 4;
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

    private double mLeftTargetAngle = LEFT_CLIMB_OUT_ANGLE;
    private double mRightTargetAngle = RIGHT_CLIMB_OUT_ANGLE;
    // Constructor
    public ClimbSubsystem()
    {
        mLeftClimbMotor = new TalonFX(LEFT_MOTOR); // Move these to constants at the top, DONE
        mRightClimbMotor = new TalonFX(RIGHT_MOTOR); // Two krakens using TalonFX motor controllers
        mGrabberMotor = new SparkMax(GRABBER_MOTOR, MotorType.kBrushless);  // Two krakens using TalonFX motor controllers

        mLeftClimbEncoder = new DutyCycleEncoder(LEFT_ENCODER);
        mRightClimbEncoder = new DutyCycleEncoder(RIGHT_ENCODER);

        mLeftClimbPID = new PIDController(LEFT_CLIMB_P, LEFT_CLIMB_I, LEFT_CLIMB_D);
        mRightClimbPID = new PIDController(RIGHT_CLIMB_P, RIGHT_CLIMB_I, RIGHT_CLIMB_D);
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

    public double getGrabberVoltage()
    {
        return mGrabberVoltage;
    }

    public void openClimber()
    {
        mLeftTargetAngle = LEFT_CLIMB_OUT_ANGLE;
        mRightTargetAngle = RIGHT_CLIMB_OUT_ANGLE;
    }

    public void closeClimber()
    {
        mLeftTargetAngle = LEFT_CLIMB_IN_ANGLE;
        mRightTargetAngle = RIGHT_CLIMB_IN_ANGLE;
    }

    public double getLeftClimbAngle()
    {
        return mLeftClimbEncoder.get();
    }

    public double getRightClimbAngle()
    {
        return mRightClimbEncoder.get();
    }
    // Run constantly to ensure that voltage is what it should be
    @Override
    public void periodic()
    {
        mLeftClimbVoltage = mLeftClimbPID.calculate(getLeftClimbAngle(), mLeftTargetAngle);
        mRightClimbVoltage = mLeftClimbPID.calculate(getRightClimbAngle(), mRightTargetAngle);

        mLeftClimbMotor.setVoltage(mLeftClimbVoltage);
        mRightClimbMotor.setVoltage(mRightClimbVoltage);
        
        SmartDashboard.putNumber("Left Climb Voltage", mLeftClimbVoltage);
        SmartDashboard.putNumber("Right Climb Voltage", mRightClimbVoltage);
        SmartDashboard.putNumber("Grabber Voltage", mGrabberVoltage);
        
    }
}
