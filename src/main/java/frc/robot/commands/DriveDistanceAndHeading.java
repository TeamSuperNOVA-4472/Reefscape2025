package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveDistanceAndHeading extends Command {
    private final SwerveSubsystem mSwerveSubsystem;

    private final Translation2d mTargetTranslation;
    private final double mTargetRotation;
    private final ProfiledPIDController mGyroController = new ProfiledPIDController(
        0.1,
        0,
        0.001,
        new Constraints(720,360));
    private final ProfiledPIDController mXController = new ProfiledPIDController(
        2,
        0,
        0,
        new Constraints(
            1.0,
            1.0
        ));
    private final ProfiledPIDController mYController = new ProfiledPIDController(
        2,
        0,
        0,
        new Constraints(
            1.0,
            1.0
        ));
    private final Timer mTimer = new Timer();
    private double mPrevTime = 0;
    private double mCurrTime = 0;
    private Translation2d mEstDistanceTraveled = new Translation2d();
    private double mNewHeading;

    public DriveDistanceAndHeading(SwerveSubsystem pSwerveSubsystem, Translation2d pTranslation, double pRotation) {
        mSwerveSubsystem =pSwerveSubsystem;
        mTargetTranslation = pTranslation;
        mTargetRotation = pRotation;
        mGyroController.enableContinuousInput(0, 360);
        addRequirements(mSwerveSubsystem);
    }

    @Override
    public void initialize() {
        mTimer.reset();
        mTimer.start();
        mPrevTime = 0;
        mCurrTime = 0;
        mEstDistanceTraveled =  new Translation2d();
        mNewHeading = mSwerveSubsystem.getHeadingDegrees() + mTargetRotation;
        mGyroController.reset(mSwerveSubsystem.getHeadingDegrees());
        mXController.reset(0);
        mYController.reset(0);
    }

    @Override
    public void execute() {
        mPrevTime = mCurrTime;
        mCurrTime = mTimer.get();
        double deltaHeading = mSwerveSubsystem.getHeadingDegrees() - mNewHeading;

        if(mPrevTime != mCurrTime) {
            double deltaTime = mCurrTime - mPrevTime;

            ChassisSpeeds targetRelativeSpeeds = 
                ChassisSpeeds.fromRobotRelativeSpeeds(
                    mSwerveSubsystem.getRobotRelativeSpeeds(),
                    Rotation2d.fromDegrees(deltaHeading));

            double newX = mEstDistanceTraveled.getX()
                + targetRelativeSpeeds.vxMetersPerSecond * deltaTime;
            double newY = mEstDistanceTraveled.getY()
                + targetRelativeSpeeds.vyMetersPerSecond * deltaTime;
            mEstDistanceTraveled = new Translation2d(newX, newY);
        }
        double rotVel = mGyroController.calculate(mSwerveSubsystem.getHeadingDegrees(), mNewHeading);
        double xVel = mXController.calculate(mEstDistanceTraveled.getX(), mTargetTranslation.getX());
        double yVel = mYController.calculate(mEstDistanceTraveled.getY(), mTargetTranslation.getY());

        ChassisSpeeds newTargetSpeeds = new ChassisSpeeds(xVel, yVel, rotVel);
        ChassisSpeeds newRobotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            newTargetSpeeds,
            Rotation2d.fromDegrees(deltaHeading));

        mSwerveSubsystem.driveRobotOriented(newRobotSpeeds);
    }
}
