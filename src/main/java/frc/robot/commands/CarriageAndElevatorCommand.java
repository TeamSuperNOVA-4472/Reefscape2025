package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.objectmodels.IntakePreset;
import frc.robot.subsystems.CarriageSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class CarriageAndElevatorCommand extends SequentialCommandGroup{
    private final CarriageSubsystem mCarriageSubsystem;
    private final ElevatorSubsystem mElevatorSubsystem;
    private final IntakePreset mPreset;

    public CarriageAndElevatorCommand(
        CarriageSubsystem pCarriageSubsystem,
        ElevatorSubsystem pElevatorSubsystem,
        IntakePreset pPreset
    )
    {
        mCarriageSubsystem = pCarriageSubsystem;
        mElevatorSubsystem = pElevatorSubsystem;
        mPreset = pPreset;

        addCommands(
            //TODO: Add new command
            //new MoveCarriageToPresetCommand(mCarriageSubsystem, mPreset),
            new MoveToLevelCommand(mElevatorSubsystem, mPreset),
            new MoveCarriageToPresetCommand(mCarriageSubsystem, mPreset)
        );
    }
}
