package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.hal.HAL;


public class TalonTest {
    private static final double DELTA = 1e-2; // Acceptable delta time
    TalonFXSimState m_simMotor;

    @BeforeEach
    void setup() {
        assert HAL.initialize(500, 0);
        m_simMotor = new TalonFXSimState(new TalonFX(0));
    }

    @AfterEach
    void shutdown() {
        m_simMotor = null;
    }

    @Test
    void checkVoltage() {
        m_simMotor.setSupplyVoltage(12);
        System.out.println(m_simMotor.getLastStatusCode().isOK() ? "Set voltage properly, see? " + m_simMotor.getLastStatusCode() : "Set voltage wrong");
        System.out.println(m_simMotor.getMotorVoltage() + " voltage measure");
        assertEquals(m_simMotor.getMotorVoltage(), 12); // TODO: Why return 0?
    }
}
