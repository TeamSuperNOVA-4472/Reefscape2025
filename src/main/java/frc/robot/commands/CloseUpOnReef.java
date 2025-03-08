package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class CloseUpOnReef extends Command {
    private final SwerveSubsystem mSwerveSubsystem;

    private final ProfiledPIDController mXPID;
    private final ProfiledPIDController mYPID;

    private double prevX;
    private double prevY;

    private final Pose2d mDestination;
    
    public CloseUpOnReef(SwerveSubsystem pSwerveSubsystem, Pose2d pDestination)
    {
        mSwerveSubsystem = pSwerveSubsystem;
        mDestination = pDestination;

        mXPID = new ProfiledPIDController(10, 0, 0, new Constraints(1.25, 1.25), 0.02);
        mYPID = new ProfiledPIDController(10, 0, 0, new Constraints(1.25, 1.25), 0.02);

        mXPID.setTolerance(0.02);
        mYPID.setTolerance(0.02);

        mXPID.setGoal(mDestination.getX());
        mYPID.setGoal(mDestination.getY());
    }

    @Override
    public void execute() {
        Pose2d curPose = mSwerveSubsystem.getPose();

        double xError = curPose.getX() - prevX;
        double yError = curPose.getY() - prevY;

        ChassisSpeeds speed = new ChassisSpeeds(
            mXPID.calculate(xError),
            mYPID.calculate(yError),
            0
            // add rotation
        ); 
        
        mSwerveSubsystem.driveRobotOriented(speed);

        prevX = curPose.getX();
        prevY = curPose.getY();
    }

    @Override
    public boolean isFinished() {
        return mXPID.atGoal() && mYPID.atGoal() ? true : false;
    }
}
