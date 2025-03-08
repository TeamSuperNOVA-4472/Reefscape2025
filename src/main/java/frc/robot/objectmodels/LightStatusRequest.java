package frc.robot.objectmodels;

public class LightStatusRequest
{
    public LightState state;
    public int priority;
    public boolean active;

    public LightStatusRequest(LightState state, int priority)
    {
        this.state = state;
        this.priority = priority;
        active = true;
    }
}
