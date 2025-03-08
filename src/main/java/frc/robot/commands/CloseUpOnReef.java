package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class CloseUpOnReef extends Command {
    private final SwerveSubsystem mSwerveSubsystem;

    private final PIDController mXPID;
    private final PIDController mYPID;

    private final Pose2d mDestination;
    
    public CloseUpOnReef(SwerveSubsystem pSwerveSubsystem, Transform2d pTransform)
    {
        mSwerveSubsystem = pSwerveSubsystem;

        mDestination = mSwerveSubsystem.getPose().transformBy(pTransform);

        mXPID = new PIDController(1, 0, 10);
        mYPID = new PIDController(1, 0, 10);

        mXPID.setTolerance(0.05);
        mYPID.setTolerance(0.05);

        addRequirements(mSwerveSubsystem);
    }

    @Override
    public void execute() {
        Pose2d curPose = mSwerveSubsystem.getPose();

        ChassisSpeeds speed = new ChassisSpeeds(
            -mXPID.calculate(curPose.getX(), mDestination.getX()),
            -mYPID.calculate(curPose.getY(), mDestination.getY()),
            0
            // add rotation
        );
        SmartDashboard.putNumber("Error X: ", mXPID.getPositionError());
        SmartDashboard.putNumber("Error Y: ", mYPID.getPositionError());
        
        mSwerveSubsystem.driveFieldOriented(speed);
    }

    @Override
    public boolean isFinished() {
        return mXPID.atSetpoint() && mYPID.atSetpoint();
    }
}
