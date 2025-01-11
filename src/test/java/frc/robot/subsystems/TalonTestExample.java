package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.unmanaged.Unmanaged;

import edu.wpi.first.hal.HAL;
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
        m_sub = new ExampleSubsystem();
        m_motor = m_sub.getMotor();
        m_state = new TalonFXSimState(m_motor);
        m_state.setSupplyVoltage(12.0);
    }

    @Test
    void boolTest(){
        assertFalse(m_sub.exampleCondition());
    }

    @Test
    void voltTest(){
        m_sub.setMotorVoltage(3.0);
        sleep(50);
        assertEquals(3.0, m_state.getMotorVoltage(), DELTA);
    }

    @AfterEach
    void shutdown(){
        m_sub.close();
    }
}
