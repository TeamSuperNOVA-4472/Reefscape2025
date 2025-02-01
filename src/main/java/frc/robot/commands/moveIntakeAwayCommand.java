package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.objectmodels.IntakePresets;
import frc.robot.subsystems.CarriageSubsystem;

public class MoveIntakeAwayCommand extends Command
{
        private final CarriageSubsystem mCarriageSubsystem;

        public MoveIntakeAwayCommand(CarriageSubsystem pCarriageSubsystem)
        {
                mCarriageSubsystem = pCarriageSubsystem;    
        }

        @Override
        public void initialize() 
        {
                mCarriageSubsystem.setActiveWristPreset(IntakePresets.kAway);
        }
}
