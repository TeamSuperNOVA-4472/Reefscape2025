package frc.robot.commands.Vision;

import java.util.Optional;
import java.util.function.Supplier;

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
    public static enum Direction {
        LEFT,
        RIGHT,
        MIDDLE
    };

    private final SwerveSubsystem mSwerveSubsystem;

    private final PIDController mXPID;
    private final PIDController mYPID;
    private final PIDController mGyroPID;

    private final Pose2d mDestination;
    private final Supplier<Direction> mDirection;

    private Pose2d mFixedDestination;
    
    public CloseUpOnReef(SwerveSubsystem pSwerveSubsystem, Pose2d pDestination, Supplier<Direction> pDirection)
    {
        mSwerveSubsystem = pSwerveSubsystem;
        mDirection = pDirection;

        mDestination = pDestination;
        mFixedDestination = mDestination.transformBy(getTransform(mDirection.get()));

        mXPID = new PIDController(5, 0, 0.2);
        mYPID = new PIDController(5, 0, 0.2);
        mGyroPID = new PIDController(5, 0, 0);

        mGyroPID.enableContinuousInput(0, 360);

        mXPID.setTolerance(0.02);
        mYPID.setTolerance(0.02);
        mGyroPID.setTolerance(0.02);

        addRequirements(mSwerveSubsystem);
    }

    @Override
    public void execute() {
        Pose2d curPose = mSwerveSubsystem.getPose();

        ChassisSpeeds speed = new ChassisSpeeds(
            -mXPID.calculate(curPose.getX(), mFixedDestination.getX()),
            -mYPID.calculate(curPose.getY(), mFixedDestination.getY()),
            mGyroPID.calculate(curPose.getRotation().getRadians(), mFixedDestination.getRotation().getRadians())
            // add rotation
        );
        SmartDashboard.putNumber("Error X: ", mXPID.getPositionError());
        SmartDashboard.putNumber("Error Y: ", mYPID.getPositionError());
        SmartDashboard.putNumber("Error Rotation: ", mGyroPID.getPositionError());
        
        mFixedDestination = mDestination.transformBy(getTransform(mDirection.get()));
        mSwerveSubsystem.driveFieldOriented(speed);
    }

    private Transform2d getTransform(Direction endDirection)
    {
        switch (endDirection)
        {
            case LEFT:
                return new Transform2d(0.65, -0.25, Rotation2d.k180deg);
            case RIGHT:
                return new Transform2d(0.65, 0.25, Rotation2d.k180deg);
            default:
                return new Transform2d(0.65, 0, Rotation2d.k180deg);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
        // mXPID.atSetpoint() && mYPID.atSetpoint() && mGyroPID.atSetpoint();
    }
}
