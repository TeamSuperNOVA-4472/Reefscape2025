package frc.robot.commands.Presets;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveCarriageToPresetCommand;
import frc.robot.commands.MoveToLevelCommand;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class AlgaeBarge extends SequentialCommandGroup{
    public AlgaeBarge(ElevatorSubsystem pElevator, CarriageSubsystem pCarriage)
    {
        addCommands(
            new StowCarriagePositionAlgae(pCarriage, pElevator),
            new MoveToLevelCommand(pElevator, ElevatorSubsystem.kPresetBarge),
            new MoveCarriageToPresetCommand(pCarriage, CarriageSubsystem.armAlgaeBarge, CarriageSubsystem.wristAlgaeBarge)
        );
    }    
}
