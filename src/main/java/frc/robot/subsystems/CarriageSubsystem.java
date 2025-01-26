package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.objectmodels.IntakePresets;

public class CarriageSubsystem extends SubsystemBase 
{

    public static final int armMotorId = 1;

    public static final int wristMotorId = 1;

    public static final int armBottomSwitch = 1;

    public static final int armTopSwitch = 1;

    public static final int wristSwitch = 1;

    public static final int armP = 1;

    public static final int armI = 1;

    public static final int armD = 1;

    public static final int wristP = 1;

    public static final int wristI = 1;

    public static final int wristD = 1;

    private static final double armPresetKAway = 0.0;

    private static final double armPresetGroundPickup = 1.0;

    private static final double armPresetL1 = 2.0;

    private static final double armPresetL2 = 3.0;

    private static final double armPresetL3 = 4.0;

    private static final double armPresetL4 = 4.0;

    private static final double wristPresetKAway = 0.0;

    private static final double wristPresetGroundPickup = 1.0;

    private static final double wristPresetL1 = 1.0;

    private static final double wristPresetL2 = 1.0;

    private static final double wristPresetL3 = 1.0;

    private static final double wristPresetL4 = 1.0;

    private TalonFX armMotor;

    private TalonFX wristMotor;

    private DigitalInput armBottom;

    private DigitalInput armTop;

    private DigitalInput wristLimit;

    private Optional<IntakePresets> activePreset = Optional.empty();

    private PIDController armPID;

    private PIDController wristPID;

    public CarriageSubsystem() 
    {
        armMotor = new TalonFX(armMotorId);

        wristMotor = new TalonFX(wristMotorId);

        armBottom = new DigitalInput(armBottomSwitch);

        armTop = new DigitalInput(armTopSwitch);

        wristLimit = new DigitalInput(wristSwitch);

        armPID = new PIDController(armP, armI, armD);

        wristPID = new PIDController(wristP, wristI, wristD);
    }

    public void stopArm() 
    {
        armMotor.stopMotor();
    }

    public void stopWrist() 
    {
        wristMotor.stopMotor();
    }

    public void setArmVoltage(double voltage) 
    {
        armMotor.setVoltage(voltage);
    }

    public void setWristVoltage(double voltage) 
    {
        wristMotor.setVoltage(voltage);
    }

    public boolean isArmAtBottom() 
    {
        return armBottom.get();
    }

    public boolean isArmAtTop() 
    {
        return armTop.get();
    }

    public boolean isWristAtLimit() 
    {
        return wristLimit.get();
    }

    // TODO: Combine these into one?
    public void setActiveArmPreset(IntakePresets preset) 
    {
        activePreset = Optional.of(preset);
    }

    public void setActiveWristPreset(IntakePresets preset) 
    {
        activePreset = Optional.of(preset);
    }

    public double getArmCurrentPosition() 
    {
        return armMotor.getPosition().getValueAsDouble();
    }

    public double getArmSetpoint() 
    {
        return armPID.getSetpoint();
    }

    public double getWristCurrentPosition() 
    {
        return wristMotor.getPosition().getValueAsDouble();
    }

    public double getWristSetpoint() 
    {
        return wristPID.getSetpoint();
    }

    @Override
    public void periodic() 
    {

        if (activePreset.isEmpty())
        {
            return;
        }

        else if (activePreset.get() == IntakePresets.kAway) 
        {
            armPID.setSetpoint(armPresetKAway);
            wristPID.setSetpoint(wristPresetKAway);
        } 
        
        else if (activePreset.get() == IntakePresets.kGroundPickup) 
        {
            armPID.setSetpoint(armPresetGroundPickup);
            wristPID.setSetpoint(wristPresetGroundPickup);
        } 
        
        else if (activePreset.get() == IntakePresets.kScoreL1) 
        {
            armPID.setSetpoint(armPresetL1);
            wristPID.setSetpoint(wristPresetL1);
        } 
        
        else if (activePreset.get() == IntakePresets.kScoreL2) 
        {
            armPID.setSetpoint(armPresetL2);
            wristPID.setSetpoint(wristPresetL2);
        } 

        else if (activePreset.get() == IntakePresets.kScoreL3) 
        {
            armPID.setSetpoint(armPresetL3);
            wristPID.setSetpoint(wristPresetL3);
        } 

        else if (activePreset.get() == IntakePresets.kScoreL4) 
        {
            armPID.setSetpoint(armPresetL4);
            wristPID.setSetpoint(wristPresetL4);
        } 
        
        else 
        {
            return;
        }

        double armCurrentPosition = armMotor.getPosition().getValueAsDouble();

        double armSpeed = armPID.calculate(armCurrentPosition);

        armMotor.set(armSpeed);

        double wristCurrentPosition = wristMotor.getPosition().getValueAsDouble();

        double wristSpeed = wristPID.calculate(wristCurrentPosition);

        wristMotor.set(wristSpeed);
    }
}
