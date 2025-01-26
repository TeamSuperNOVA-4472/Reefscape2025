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

    public static final int wristSwitch = 0;

    public static final int armP = 1;

    public static final int armI = 1;

    public static final int armD = 1;

    public static final int wristP = 1;

    public static final int wristI = 1;

    public static final int wristD = 1;

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

    @Override
    public void periodic() 
    {
        double armDesiredPosition;

        if (activeArmPreset == 0) 
        {
            armDesiredPosition = 0;
        } 
        
        else if (activeArmPreset == 1) 
        {
            armDesiredPosition = 1;
        } 
        
        else if (activeArmPreset == 2) 
        {
            armDesiredPosition = 2;
        } 
        
        else 
        {
            return;
        }

        armPID.setSetpoint(armDesiredPosition);

        double armCurrentPosition = armMotor.getPosition().getValueAsDouble();

        double armSpeed = armPID.calculate(armCurrentPosition);

        armMotor.set(armSpeed);

        double wristDesiredPosition;

        if (activeWristPreset == 0) 
        {
            wristDesiredPosition = 0;
        } 
        
        else if (activeWristPreset == 1) 
        {
            wristDesiredPosition = 1;
        } 
        
        else 
        {
            return;
        }
    
        wristPID.setSetpoint(wristDesiredPosition);
    
        double wristCurrentPosition = wristMotor.getPosition().getValueAsDouble();

        double wristSpeed = wristPID.calculate(wristCurrentPosition);

        wristMotor.set(wristSpeed);
    }
}
