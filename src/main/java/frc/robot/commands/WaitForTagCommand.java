package frc.robot.commands;

import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

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
    private Supplier<Integer> specificTagSupplier;
    private Timer timer;

    public WaitForTagCommand(VisionSubsystem vision, int tagId, double maxTime)
    {
        mVision = vision;
        mMaxTime = maxTime;
        specificTagSupplier = () -> tagId;
        result = Optional.empty();
        callback = Optional.empty();
        timer = new Timer();

        addRequirements(vision);
    }
    // More constructors can be added if you want.
    public WaitForTagCommand(VisionSubsystem vision, Supplier<Integer> tagSupplier, double maxTime, Consumer<PhotonTrackedTarget> callback)
    {
        mVision = vision;
        mMaxTime = maxTime;
        specificTagSupplier = tagSupplier;
        result = Optional.empty();
        this.callback = Optional.of(callback);
        timer = new Timer();

        addRequirements(vision);
    }

    @Override
    public void initialize()
    {
        result = Optional.empty();
        timer.restart();
    }

    @Override
    public void execute()
    {
        int specificTag = specificTagSupplier.get();
        Optional<PhotonTrackedTarget> possible = mVision.getTargetInView();
        if (possible.isEmpty()) return; // No target seen.
        else if (specificTag == -1 || specificTag == possible.get().fiducialId)
        {
            result = possible;
            if (callback.isPresent())
            {
                callback.get().accept(result.get());
            }
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        timer.stop();
    }

    @Override
    public boolean isFinished()
    {
        return result.isPresent() ||
               timer.hasElapsed(mMaxTime);
    }
}
