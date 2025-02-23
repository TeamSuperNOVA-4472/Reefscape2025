package frc.robot.objectmodels;

import edu.wpi.first.math.geometry.Transform3d;

public class CameraInfo
{
    private Transform3d offset;
    private String name;

    public CameraInfo(String name, Transform3d offset)
    {
        this.name = name;
        this.offset = offset;
    }

    public Transform3d getOffset()
    {
        return offset;
    }
    public String getName()
    {
        return name;
    }
}
