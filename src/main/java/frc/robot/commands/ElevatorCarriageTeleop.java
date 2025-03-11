package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.objectmodels.CarriagePreset;
import frc.robot.subsystems.ElevatorCarriageSubsystem;

public class ElevatorCarriageTeleop extends Command 
{
    private final ElevatorCarriageSubsystem mElevatorCarriageSubsystem;

    private final XboxController mController;

    public ElevatorCarriageTeleop(XboxController pController)
    {
        mElevatorCarriageSubsystem = ElevatorCarriageSubsystem.instance();
        mController = pController;

        addRequirements(mElevatorCarriageSubsystem);

        mElevatorCarriageSubsystem.resetElevatorEncoder();
    }

    @Override
    public void initialize()
    {
        mElevatorCarriageSubsystem.resetPID();
        //mElevatorCarriageSubsystem.stop();
        mElevatorCarriageSubsystem.setPreset(CarriagePreset.kStowCoral);
    }

    @Override
    public void execute()
    {

        if(mElevatorCarriageSubsystem.getActivePreset().isEmpty()) {
            mElevatorCarriageSubsystem.setManualElevatorVoltage(-mController.getLeftY() * 6);
            mElevatorCarriageSubsystem.setManualArmVoltage(-mController.getRightY() * 12);
            mElevatorCarriageSubsystem.setManualWristVoltage(mController.getRightX() * 2);
        }
    
    }
}
