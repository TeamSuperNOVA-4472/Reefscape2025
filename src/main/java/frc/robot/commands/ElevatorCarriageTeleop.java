package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorCarriageSubsystem;

public class ElevatorCarriageTeleop extends Command 
{
    private final ElevatorCarriageSubsystem mElevatorCarriageSubsystem;
    private final XboxController mController;
    public ElevatorCarriageTeleop(ElevatorCarriageSubsystem pElevatorCarriageSubsystem, XboxController pController)
    {
        mElevatorCarriageSubsystem = pElevatorCarriageSubsystem;
        mController = pController;
        addRequirements(pElevatorCarriageSubsystem);
    }

    @Override
    public void execute()
    {
        /*if(mIntakePresets == IntakePresets.kScoreL1)
        {
            //do level one command
        }
        else if(mIntakePresets == IntakePresets.kScoreL2)
        {
            //do level two command
        }
        else if(mIntakePresets == IntakePresets.kScoreL3)
        {
            //do level three command
        }
        else if(mIntakePresets == IntakePresets.kScoreL4)
        {
            //do level four command
        }
        else if(mIntakePresets == IntakePresets.kAway)
        {
            //do away command
        }
        else if(mIntakePresets == IntakePresets.kGroundPickup)
        {
            //do ground pickup command
        }*/
    }
}
