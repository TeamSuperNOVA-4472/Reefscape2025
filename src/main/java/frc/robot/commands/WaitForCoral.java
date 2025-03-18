package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class WaitForCoral extends Command{
    public WaitForCoral(){
        addRequirements(IntakeSubsystem.kInstance);
    }
    @Override
    public boolean isFinished(){
        return IntakeSubsystem.kInstance.hasCoral();
    }
}
