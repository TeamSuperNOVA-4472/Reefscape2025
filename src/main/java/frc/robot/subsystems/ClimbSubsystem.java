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
    public static final ClimbSubsystem kInstance = new ClimbSubsystem();

    // THE CONSTANTS
    private static final int CLIMB_MOTOR = 12;
    private static final int CLIMB_ENCODER = 3; //either 0, 1, 2, or 3 for both encoders

    private static final double CLIMB_OUT_ANGLE = 0;
    private static final double CLIMB_IN_ANGLE = 0;

    private static final int CLIMB_P = 0;
    private static final int CLIMB_I = 0;
    private static final int CLIMB_D = 0;

    private static final double CLIMB_OFFSET = 150;
    private static final double MAX_CLIMB = 190;
    private static final double MIN_CLIMB = -17;
    
    private final TalonFX mClimbMotor;

    private final PIDController mClimbPID;

    private final DutyCycleEncoder mClimbEncoder;

    private double mClimbVoltage = 0;

    private boolean mClosing; // Don't have time for a better implementation.
    
    private double mRightTargetAngle = CLIMB_OUT_ANGLE;
    // Constructor
    private ClimbSubsystem()
    {
        mClimbMotor = new TalonFX(CLIMB_MOTOR); // Two krakens using TalonFX motor controllers

        mClimbEncoder = new DutyCycleEncoder(CLIMB_ENCODER);

        mClimbPID = new PIDController(CLIMB_P, CLIMB_I, CLIMB_D);

        mClosing = false;
    }

    
    public double getClimbVoltage()
    {
        return mClimbVoltage;
    }
    
    public void setClimbVoltage(double pNewVoltage)
    {
        mClimbVoltage = pNewVoltage; 
    }

    /*public void moveOutClimber()
    {
        mLeftTargetAngle = LEFT_CLIMB_OUT_ANGLE;
        mRightTargetAngle = RIGHT_CLIMB_OUT_ANGLE;
        mClosing = false;
    }

    public void moveInClimber()
    {
        mLeftTargetAngle = LEFT_CLIMB_IN_ANGLE;
        mRightTargetAngle = RIGHT_CLIMB_IN_ANGLE;
        mClosing = true;
    }

    public boolean isClosingOrClosed()
    {
        return mClosing;
    }

    public double getClimbAngle()
    {

        double value = mClimbEncoder.get() * 360 + CLIMB_OFFSET;
        if (value >= 250) value -= 360;
        return value;
    }*/
    // Run constantly to ensure that voltage is what it should be
    
    @Override
    public void periodic()
    {
        
        
        SmartDashboard.putNumber("Climb Voltage", mClimbVoltage);
        //SmartDashboard.putNumber("Left Climb angle", getClimbAngle());        
    }
}
