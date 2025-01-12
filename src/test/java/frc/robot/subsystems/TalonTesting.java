package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;

public class TalonTesting implements AutoCloseable {
    static final double DELTA = 1e-2; // acceptable deviation range
 
    TalonFX m_fx;
    TalonFXSimState m_fxSim;
 
    @Override
    public void close() {
       /* destroy our TalonFX object */
       m_fx.close();
    }
 
    @BeforeEach
    public void constructDevices() {
       assert HAL.initialize(500, 0);
 
       /* create the TalonFX */
       m_fx = new TalonFX(0);
       m_fxSim = m_fx.getSimState();
 
       /* enable the robot */
       DriverStationSim.setEnabled(true);
       DriverStationSim.notifyNewData();
 
       /* delay ~100ms so the devices can start up and enable */
       Timer.delay(0.100);
    }
 
    @AfterEach
    void shutdown() {
       close();
    }
 
    @Test
    public void robotIsEnabled() {
       /* verify that the robot is enabled */
       assertTrue(DriverStation.isEnabled());
    }
 
    @Test
    public void motorDrives() {
       /* set the voltage supplied by the battery */
       m_fxSim.setSupplyVoltage(16);
 
       StatusSignal<Double> dutyCycle = m_fx.getDutyCycle();
 
       /* wait for a fresh duty cycle signal */
       dutyCycle.waitForUpdate(0.100);
       /* verify that the motor output is zero */
       assertEquals(dutyCycle.getValue(), 0.0, DELTA);
 
       /* request 100% output */
       m_fx.setControl(new DutyCycleOut(1.0));
       /* wait for the control to apply */
       Timer.delay(0.020);
 
       /* wait for a new duty cycle signal */
       dutyCycle.waitForUpdate(1.00);
       /* verify that the motor output is 1.0 */
       double outVal = dutyCycle.getValue();
       System.out.println(outVal);
       assertEquals(outVal, 1.0, DELTA);
    } 
 }