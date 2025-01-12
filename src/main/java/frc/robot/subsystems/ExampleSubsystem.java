// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// Represents an example subsystem. It will be removed once some actual
// subsystems get created.
public class ExampleSubsystem extends SubsystemBase
{
    private RobotConfig sadf;
    private static TalonFX mEx;
    // Set up the example subsystem.
    public ExampleSubsystem() {
        mEx = new TalonFX(1/* , new CANBus("rio")*/);
        /*mEx.getConfigurator().apply(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(20)
                .withSupplyCurrentLimit(20)
                .withSupplyCurrentLowerLimit(0)
                .withSupplyCurrentLowerTime(0));
        mEx.getConfigurator().apply(
            new VoltageConfigs()
                .withPeakForwardVoltage(12)
                .withPeakReverseVoltage(-12));
        mEx.setNeutralMode(NeutralModeValue.Brake);*/
    }

    public TalonFX getMotor(){
        return mEx;
    }
    // An example method that would do something to the subsystem's
    // physical state.
    public void exampleMethod(double value) { }

    // An example command that is created when this method gets called.
    public Command exampleMethodCommand()
    {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
                () -> {
                    /* one-time action goes here */
                });
    }

    // An example method that determines a condition related to the state of
    // the subsystem. Can be used to trigger other things.
    public boolean exampleCondition()
    {
        // Query some boolean state, such as a digital sensor.
        return false;
    }

    public void setMotorVoltage(double volt){
        mEx.setVoltage(volt);
    }

    // Run continously. In a perfect situation, this runs every 20 ms,
    // or 50 times per second.
    @Override
    public void periodic() { }

    // Runs continously, but only when simulated. Runs 50 times per second.
    @Override
    public void simulationPeriodic() { }

    public void close() {
        mEx.close();
    }
}
