package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Represents a command that never ends. Must be cancelled.
 */
public class ForeverCommand extends Command
{
    public ForeverCommand() { }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
