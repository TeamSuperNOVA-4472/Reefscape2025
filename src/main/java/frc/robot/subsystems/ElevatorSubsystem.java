package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.objectmodels.IntakePresets;

public class ElevatorSubsystem extends SubsystemBase
{
    public static final int kLeftElevatorMotorID = 12;
    public static final int kRightElevatorMotorID = 13;

    public static final int kBottomSwitchChannel = 0;
    public static final int kTopSwitchChannel = 1;

    public static final double kPresetAway = 0;
    public static final double kPresetGroundPickup = 0;
    public static final double kPresetL1 = 1;
    public static final double kPresetL2 = 2;
    public static final double kPresetL3 = 3;
    public static final double kPresetL4 = 4;

    public static final double kElevatorP = 0.5;
    public static final double kElevatorI = 0.0002;
    public static final double kElevatorD = 0.15;

    private final TalonFX mElevatorLeft;
    private final TalonFX mElevatorRight;

    private Optional<IntakePresets> activePreset = Optional.empty();

    private PIDController elevatorPID;
    private final DCMotor elevGearbox = DCMotor.getKrakenX60(2);
    private boolean isMovingUp = false;
    private boolean isMovingDown = false;

    private final Mechanism2d mech2d = new Mechanism2d(5, 5);
    private final MechanismRoot2d mech2dRoot = mech2d.getRoot("Elevator Root", 2, 0);
    private MechanismLigament2d elevatorMech2d;
    

    private TalonFXSimState elevatorMotorSim1;
    private TalonFXSimState elevatorMotorSim2;
    private ElevatorSim elevSim;

    private DigitalInput bottomSwitch;
    private DigitalInput topSwitch;

    public ElevatorSubsystem()
    {
        bottomSwitch = new DigitalInput(kBottomSwitchChannel);
        topSwitch = new DigitalInput(kTopSwitchChannel);

        mElevatorLeft = new TalonFX(kLeftElevatorMotorID, Constants.kCanivoreBusName);
        mElevatorRight = new TalonFX(kRightElevatorMotorID);
        elevatorMotorSim1 = new TalonFXSimState(mElevatorLeft);
        elevatorMotorSim2 = new TalonFXSimState(mElevatorRight);
        elevSim = new ElevatorSim(LinearSystemId.createElevatorSystem(elevGearbox, 1, 0.1, 1), elevGearbox, 1, 5, true, 1, 0.01, 0);
        elevatorPID = new PIDController(kElevatorP, kElevatorI, kElevatorD);

        elevatorMech2d = mech2dRoot.append(new MechanismLigament2d("Elevator", elevSim.getPositionMeters(), 90));
        SmartDashboard.putData("Elevator Sim", mech2d);
    }

    public void stop()
    {
        mElevatorLeft.stopMotor();
        isMovingUp = false;
        isMovingDown = false;
        activePreset = Optional.empty();
    }

    public void setVoltage(double voltage)
    {
        final double kVoltageTolerance = 0.1;

        mElevatorLeft.setVoltage(voltage);
        mElevatorRight.setVoltage(-voltage);
        activePreset = Optional.empty();

        if (voltage > kVoltageTolerance)
        {
            isMovingUp = true;
            isMovingDown = false;
        }

        else if (voltage < -kVoltageTolerance)
        {
            isMovingDown = true;
            isMovingUp = false;
        }

        else
        {
            isMovingUp = false; 
            isMovingDown = false;
        }
    }

    public boolean isAtBottom()
    {
        return bottomSwitch.get();
    }
    public boolean isAtTop()
    {
        return topSwitch.get();
    }
    public boolean isMoving()
    {
        return isMovingUp || isMovingDown;
    }

    public void setPreset(IntakePresets preset)
    {
        activePreset = Optional.of(preset);
    }

    public String getPresetAsString(){
        IntakePresets tempPreset = activePreset.orElse(IntakePresets.kGroundPickup);
        switch (tempPreset){
            case kScoreL3:
                return "ScoreL3";
            default:
                return "None";
        }
    }

    @Override
    public void simulationPeriodic() {
        // In this method, we update our simulation of what our elevator is doing
        // First, we set our "inputs" (voltages)
        elevSim.setInput(elevatorMotorSim1.getMotorVoltage());

        // Next, we update it. The standard loop time is 20ms.
        elevSim.update(0.020);

        // SimBattery estimates loaded battery voltages
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(elevSim.getCurrentDrawAmps()));
        elevatorMech2d.setLength((elevSim.getPositionMeters()));
        SmartDashboard.putNumber("ElevatorLength", elevSim.getPositionMeters());
        SmartDashboard.putString("P", this.getPresetAsString());
        SmartDashboard.putNumber("Left Motor Volatge", elevatorMotorSim1.getMotorVoltage());
        SmartDashboard.putNumber("Right Motor Volatge", elevatorMotorSim2.getMotorVoltage());
    }

    @Override
    public void periodic()
    {

        if (activePreset.isEmpty()) return; // No preset.
        else
        {
            switch (activePreset.get())
            {
                case kAway:
                    elevatorPID.setSetpoint(kPresetAway);
                    break;
                
                case kGroundPickup:
                    elevatorPID.setSetpoint(kPresetGroundPickup);
                    break;

                case kScoreL1:
                    elevatorPID.setSetpoint(kPresetL1);
                    break;
                    
                case kScoreL2:
                    elevatorPID.setSetpoint(kPresetL2);
                    break;
                
                case kScoreL3:
                    elevatorPID.setSetpoint(kPresetL3);
                    break;
                    
                case kScoreL4:
                    elevatorPID.setSetpoint(kPresetL4);
                    break;
                
                default: return; // Unknown preset. Shouldn't happen.
            }
        }
        double currentPosition;
        if (Robot.isReal()) currentPosition = mElevatorLeft.getPosition().getValueAsDouble();
        else currentPosition = elevSim.getPositionMeters();
        double newSpeed = elevatorPID.calculate(currentPosition);
        SmartDashboard.putNumber("Speed of Elevator", newSpeed);
        mElevatorLeft.set(newSpeed);
        mElevatorRight.set(-newSpeed);
    }
}
