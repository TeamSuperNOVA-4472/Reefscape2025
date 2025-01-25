package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class DoTheThingCommand extends SequentialCommandGroup
{
    public DoTheThingCommand()
    {
        addCommands(
            new InstantCommand(() -> System.out.println("Starting the thing!")),
            new WaitCommand(1.0),
            new InstantCommand(() -> System.out.println("Still doing the thing!")),
            new WaitCommand(1.0),
            new InstantCommand(() -> System.out.println("Doing another thing!")),
            new WaitCommand(1.0),
            new InstantCommand(() -> System.out.println("Thingy the thing!")),
            new WaitCommand(1.0),
            new InstantCommand(() -> System.out.println("Did the thing!")));
    }
}
