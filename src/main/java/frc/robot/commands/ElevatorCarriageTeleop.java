package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CarriageSubsystem;

public class ElevatorCarriageTeleop extends Command
{
    private final CarriageSubsystem mCarriageSubsystem;

    private final int armPresetButton;

    private final int wristPresetButton;

    public ElevatorCarriageTeleop(CarriageSubsystem carriageSubsystem, int armPresetButton, int wristPresetButton) 
    {
        mCarriageSubsystem = carriageSubsystem;

        this.armPresetButton = armPresetButton;

        this.wristPresetButton = wristPresetButton;

        addRequirements(mCarriageSubsystem);
    }

    @Override
    public void initialize() 
    {
        if (armPresetButton == 0) 
        {
            mCarriageSubsystem.setActiveArmPreset(0);
        } 
        
        else if (armPresetButton == 1) 
        {
            mCarriageSubsystem.setActiveArmPreset(1);
        } 
        
        else if (armPresetButton == 2) 
        {
            mCarriageSubsystem.setActiveArmPreset(2);
        } 
        
        else if (armPresetButton == 3) 
        {
            mCarriageSubsystem.setActiveArmPreset(3);
        }

        if (wristPresetButton == 0) 
        {
            mCarriageSubsystem.setActiveWristPreset(0);
        } 
        
        else if (wristPresetButton == 1) 
        {
            mCarriageSubsystem.setActiveWristPreset(1);
        }
    }

    @Override
    public void end(boolean stopping) 
    {
        mCarriageSubsystem.stopArm();

        mCarriageSubsystem.stopWrist();
    }

    @Override
    public boolean isFinished() 
    {
        boolean armAtTarget = mCarriageSubsystem.getArmCurrentPosition() == mCarriageSubsystem.getArmSetpoint();

        boolean wristAtTarget = mCarriageSubsystem.getWristCurrentPosition() == mCarriageSubsystem.getWristSetpoint();

        return armAtTarget && wristAtTarget;
    }
}