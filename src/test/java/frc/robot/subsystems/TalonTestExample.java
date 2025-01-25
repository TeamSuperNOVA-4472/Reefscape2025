/*package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.unmanaged.Unmanaged;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;

public class TalonTestExample {
    private static final double DELTA = 1e-2;
    ExampleSubsystem m_sub;
    TalonFXSimState m_state;
    TalonFX m_motor;

    private void sleep(long millis) {
        try {
            for(int i = 0; i < millis; i += 10) {
                Unmanaged.feedEnable(20);
                Thread.sleep(10);
            }
        } catch(InterruptedException ex) {
            ex.printStackTrace();
        }
    }

    @BeforeEach
    void setup(){
        assert HAL.initialize(500, 0);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        m_sub = new ExampleSubsystem();
        m_motor = m_sub.getMotor();
        m_state = m_motor.getSimState();
        m_state.setSupplyVoltage(16);
        Timer.delay(0.100);
    }
    
    @Test
    void boolTest(){
        assertFalse(m_sub.exampleCondition());
    }

    @Test
    void voltTest(){
        m_sub.setMotorVoltage(6);
        sleep(75);
        double voltage = m_state.getMotorVoltage();
        System.out.println(voltage);
        System.out.println(m_state.getSupplyCurrent());
        assertEquals(6, voltage, DELTA);
    }

    @AfterEach
    void shutdown(){
        m_sub.close();
    }
}
*/