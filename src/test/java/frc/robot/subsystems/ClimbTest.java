package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertNotNull;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;

public class ClimbTest {
    static final double DELTA = 1e-2; // acceptable deviation range
    ClimbSubsystem m_climb;
    PWMSim m_simMotor;
    DoubleSolenoidSim m_simPiston;

    @BeforeEach // this method will run before each test
    void setup() {
        assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
        m_climb = new ClimbSubsystem(); // create our intake
        m_simMotor = new PWMSim(IntakeSubsystem.kIntakeMotorId); // create our simulation PWM motor controller
  }

  @SuppressWarnings("PMD.SignatureDeclareThrowsException")
  @AfterEach // this method will run after each test
  void shutdown() throws Exception {
    //m_climb.close(); // destroy our intake object
  }

  @Test
  void testIntake() {
    assertNotNull(m_climb);
  }

}
