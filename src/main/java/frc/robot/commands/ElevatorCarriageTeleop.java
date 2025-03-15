package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.objectmodels.CarriagePreset;
import frc.robot.subsystems.ElevatorCarriageSubsystem;

// FIXME: THIS IS A TEST FILE, PLEASE REPLACE WITH FARIS'S PRESETTELEOP COMMAND.
public class ElevatorCarriageTeleop extends Command 
{
    private final ElevatorCarriageSubsystem mElevatorCarriageSubsystem;

    private final XboxController mController;
    
    private Command activeCommand;

    public ElevatorCarriageTeleop(XboxController pController)
    {
        mElevatorCarriageSubsystem = ElevatorCarriageSubsystem.kInstance;
        mController = pController;

        addRequirements(mElevatorCarriageSubsystem);

        mElevatorCarriageSubsystem.resetElevatorEncoder();
    }

    @Override
    public void initialize()
    {
        mElevatorCarriageSubsystem.resetPID();
        //mElevatorCarriageSubsystem.stop();
        //mElevatorCarriageSubsystem.setPreset(CarriagePreset.kCoralL2);
        /*activeCommand = new SequentialCommandGroup(
            //new moveToLevelSafe(CarriagePreset.kStowCoral),
            new MoveCarriageToPresetCommand(CarriagePreset.kStowCoral)
        );
        activeCommand.schedule();*/
    }

    @Override
    public void execute()
    {

        /*if(mElevatorCarriageSubsystem.getActivePreset().isEmpty()) {
            mElevatorCarriageSubsystem.setManualElevatorVoltage(-mController.getLeftY() * 6);
            mElevatorCarriageSubsystem.setManualArmVoltage(-mController.getRightY() * 12);
            mElevatorCarriageSubsystem.setManualWristVoltage(mController.getRightX() * 2);
        }*/
    
    }
}
