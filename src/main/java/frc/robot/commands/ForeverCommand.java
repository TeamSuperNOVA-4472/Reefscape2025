package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Represents a command that never ends. Must be cancelled.
 */
public class ForeverCommand extends Command
{
    public ForeverCommand()
    {

    }

    @Override
    public void initialize() {
        System.out.println("hey I ran. this is bad. hopefully you dont see this");
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
