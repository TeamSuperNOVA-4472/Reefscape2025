package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbTeleop extends Command 
{
    // A lot of this logic might be better served in the climb subsystem,
    // not the teleop. But for now it will do.
    
    // By default, the mechanism goes to the stow position.
    // When the right trigger is pressed, it goes to the ready position.
    // And when the left trigger is pressed, it goes to the climbing position.

    public static final double kAngleStow = 33;
    public static final double kAngleClimb = 30; // Used to be 22, ambitious is 11.
    public static final double kAngleReady = 135.5;

    public static final double kTolerance = 3; // The result will be correct plus or minus this number in degrees.

    private static final double kClimbVoltage = 12; // Max is 12v.

    private final ClimbSubsystem mClimbSubsystem;

    private final Supplier<Boolean> mClimbReady, mClimbActivate;

    private ClimbState mState = ClimbState.kStow;

    public ClimbTeleop(Supplier<Boolean> pClimbReady, Supplier<Boolean> pClimbActivate)
    {
        mClimbSubsystem = ClimbSubsystem.kInstance;

        mClimbReady = pClimbReady;
        mClimbActivate = pClimbActivate;
        
        addRequirements(mClimbSubsystem);
    }    

    @Override
    public void execute()
    {
        // Determine state.
        if (mClimbReady.get())
        {
            if (mState == ClimbState.kReady) mState = ClimbState.kPause;
            else mState = ClimbState.kReady;
        }
        else if (mClimbActivate.get())
        {
            if (mState == ClimbState.kClimb) mState = ClimbState.kPause;
            else mState = ClimbState.kClimb;
        }

        // Determine position based on state.
        double currentRot = mClimbSubsystem.getClimbAngleDegrees();
        double desiredRot;
        switch (mState)
        {
            case kStow: desiredRot = kAngleStow; break;
            case kReady: desiredRot = kAngleReady; break;
            case kClimb: desiredRot = kAngleClimb; break;
            case kPause: desiredRot = currentRot; break;
            default: desiredRot = 90; break; // Default state, should never happen, but just in case it's a safe value to go to.
        }

        // Then move the motor in the direction needed (if out of tolerance).
        double diff = desiredRot - currentRot;

        if (Math.abs(diff) >= kTolerance) // Out of tolerance, actually move.
        {
            // Move in direction. signum() is the sign of the number,
            // and is a shorthand for moving in the negative direction
            // if the distance is negative. Saves me an if statement.
            mClimbSubsystem.setVoltage(kClimbVoltage * -Math.signum(diff));
        }

        else mClimbSubsystem.setVoltage(0);
    }

    private enum ClimbState
    {
        kStow,
        kReady,
        kClimb,
        kPause
    }
}
