package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorCarriageSubsystem extends SubsystemBase
{
    private final ElevatorSubsystem mElevatorSubsystem;
    private final CarriageSubsystem mCarriageSubsystem;
    public ElevatorCarriageSubsystem(ElevatorSubsystem pElevatorSubsystem, CarriageSubsystem pCarriageSubsystem)
    {
        mElevatorSubsystem = pElevatorSubsystem;
        mCarriageSubsystem = pCarriageSubsystem;
    }
    
    public ElevatorSubsystem getElevatorSubsystem()
    {
        return mElevatorSubsystem;
    }

    public CarriageSubsystem getCarriageSubsystem()
    {
        return mCarriageSubsystem;
    }
}
