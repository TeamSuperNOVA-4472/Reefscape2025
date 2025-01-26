package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    private static final double armPreset0 = 0.0;

    private static final double armPreset1 = 1.0;

    private static final double armPreset2 = 2.0;

    private static final double armPreset3 = 3.0;

    private static final double wristPreset0 = 0.0;

    private static final double wristPreset1 = 1.0;

    private TalonFX armMotor;

    private TalonFX wristMotor;

    private DigitalInput armBottom;

    private DigitalInput armTop;

    private DigitalInput wristLimit;

    private int activeArmPreset = -1;

    private int activeWristPreset = -1;

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
    public void setActiveArmPreset(int preset) 
    {
        activeArmPreset = preset;
    }

    public void setActiveWristPreset(int preset) 
    {
        activeWristPreset = preset;
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
        if (activeArmPreset == 0) 
        {
            armPID.setSetpoint(armPreset0);
        } 
        
        else if (activeArmPreset == 1) 
        {
            armPID.setSetpoint(armPreset1);
        } 
        
        else if (activeArmPreset == 2) 
        {
            armPID.setSetpoint(armPreset2);
        } 
        
        else if (activeArmPreset == 3) 
        {
            armPID.setSetpoint(armPreset3);
        } 
        
        else 
        {
            return;
        }

        double armCurrentPosition = armMotor.getPosition().getValueAsDouble();

        double armSpeed = armPID.calculate(armCurrentPosition);

        armMotor.set(armSpeed);

        if (activeWristPreset == 0) 
        {
            wristPID.setSetpoint(wristPreset0);
        } 
        
        else if (activeWristPreset == 1) 
        {
            wristPID.setSetpoint(wristPreset1);
        } 
        
        else 
        {
            return;
        }

        double wristCurrentPosition = wristMotor.getPosition().getValueAsDouble();

        double wristSpeed = wristPID.calculate(wristCurrentPosition);

        wristMotor.set(wristSpeed);
    }
}
