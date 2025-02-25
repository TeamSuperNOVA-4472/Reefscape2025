package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.MoveCarriageToPresetCommand;
import frc.robot.commands.Intake.MoveToLevelCommand;
import frc.robot.commands.Intake.MoveToLevelSafe;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AlgaeProcessor extends SequentialCommandGroup {
    public AlgaeProcessor(ElevatorSubsystem pElevator, CarriageSubsystem pCarriage, IntakeSubsystem pIntakeSubsystem)
    {
        addCommands(
            //new MoveToLevelSafe(pCarriage, pElevator, pIntakeSubsystem, ElevatorSubsystem.kPresetProcessor),
            //ew MoveCarriageToPresetCommand(pCarriage, CarriageSubsystem.armAlgaeProcessor, CarriageSubsystem.wristAlgaeProcessor)
        );
    }    
}
