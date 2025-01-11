package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.system.plant.DCMotor;

public class TestExample {
    private static final double DELTA = 1e-2; // Acceptable delta time
    SparkMaxSim m_simMotor;

    @BeforeEach // Runs before each test
    void setup() {
        assert HAL.initialize(500, 0);
        m_simMotor = new SparkMaxSim(new SparkMax(1, MotorType.kBrushless), new DCMotor(1, 1, 1, 1, 1, 0));
    }

    @AfterEach // Runs after each test
    void shutdown() {
        m_simMotor = null;
    }

    @Test // Is a test, runs on build
    void exampleTest() {
        m_simMotor.setBusVoltage(120);
        assertEquals(m_simMotor.getBusVoltage(), 120.0);
    }
}
