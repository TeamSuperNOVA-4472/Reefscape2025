package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertNotNull;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.PWMSim;

public class ElevatorCarriageTest {
  static final double DELTA = 1e-2; // acceptable deviation range
  ElevatorSubsystem m_elevator;
  CarriageSubsystem m_carriageSubsystem;
  ElevatorCarriageSubsystem m_elevatorcarriageSubsystem;
  PWMSim m_simMotor1, m_simMotor2;

    @BeforeEach // this method will run before each test
  void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    m_elevator = new ElevatorSubsystem(); // create our intake
    m_carriageSubsystem = new CarriageSubsystem();
    m_elevatorcarriageSubsystem = new ElevatorCarriageSubsystem(m_elevator, m_carriageSubsystem);
    m_simMotor1 = new PWMSim(ElevatorSubsystem.kLeftElevatorMotorID); // create our simulation PWM motor controller
    m_simMotor2 = new PWMSim(ElevatorSubsystem.kRightElevatorMotorID);
  }

  @SuppressWarnings("PMD.SignatureDeclareThrowsException")
  @AfterEach // this method will run after each test
  void shutdown() throws Exception {
    m_elevator.stop();
  }

  @Test
  void elevatorCarriageTest() {
    assertNotNull(m_elevator);
    assertNotNull(m_carriageSubsystem);
    assertNotNull(m_elevatorcarriageSubsystem);
  }
}
