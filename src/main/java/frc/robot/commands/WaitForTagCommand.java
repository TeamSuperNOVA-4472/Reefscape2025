package frc.robot.commands;

import java.util.Optional;
import java.util.function.Consumer;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;

// Uses the vision subsystem to wait for a tag to appear.
public class WaitForTagCommand extends Command
{
    private final VisionSubsystem mVision;
    private final double mMaxTime;

    private Optional<Consumer<PhotonTrackedTarget>> callback;
    private Optional<PhotonTrackedTarget> result;
    private Timer timer;

    public WaitForTagCommand(VisionSubsystem vision, double maxTime)
    {
        mMaxTime = maxTime;
        mVision = vision;
        result = Optional.empty();
        timer = new Timer();
        callback = Optional.empty();

        addRequirements(vision);
    }
    public WaitForTagCommand(VisionSubsystem vision, double maxTime, Consumer<PhotonTrackedTarget> callback)
    {
        this(vision, maxTime);
        this.callback = Optional.of(callback);
    }

    @Override
    public void execute()
    {
        result = mVision.getTargetInView();
        if (result.isPresent() && callback.isPresent())
        {
            callback.get().accept(result.get());
        }
    }

    @Override
    public boolean isFinished()
    {
        return result.isPresent() ||
               timer.hasElapsed(mMaxTime);
    }
}
