package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// TODO: Actual motor ports
public class ClimbSubsystem extends SubsystemBase
{
    public static final ClimbSubsystem kInstance = new ClimbSubsystem();

    // THE CONSTANTS
    private static final double CLIMB_OFFSET_ANGLE = 0; // Used to be 166-90, why? It seems to be unneeded.
    private static final int CLIMB_MOTOR = 12;

    
    private final TalonFX mClimbMotor;

    private final DutyCycleEncoder mClimbEncoder;

    // TODO: This is incomplete. Eventually, this will mesh with the climbteleop code once its
    //       moved over here.
    private boolean mIsClimbing;

    // Constructor
    private ClimbSubsystem()
    {
        mClimbMotor = new TalonFX(CLIMB_MOTOR, "CANivore"); // Move these to constants at the top, DONE
        mClimbEncoder = new DutyCycleEncoder(4);
        SmartDashboard.putBoolean("Subsystems/ClimbSubsystem/Is Climbing", false);
    }

    public void setVoltage(double pVoltage)
    {
        mClimbMotor.setVoltage(pVoltage);
    }

    public double getClimbAngleDegrees() {
        double climbAngleDeg = mClimbEncoder.get() * 360 - CLIMB_OFFSET_ANGLE;
        return climbAngleDeg;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climb Encoder Degrees", getClimbAngleDegrees());
    }

    public boolean isClimbing()
    {
        return mIsClimbing;
    }
    public void setIsClimbing(boolean pIsClimbing)
    {
        mIsClimbing = pIsClimbing;
        SmartDashboard.putBoolean("Subsystems/ClimbSubsystem/Is Climbing", pIsClimbing);
    }
}
