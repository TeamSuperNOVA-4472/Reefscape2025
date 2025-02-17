package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertNotNull;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.ctre.phoenix6.sim.CANcoderSimState;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;

public class SwerveTest {
  static final double DELTA = 1e-2; // acceptable deviation range
  SwerveSubsystem m_swerve;
  PWMSim m_simRightFrontDriveMotor;
  PWMSim m_simRightBackDriveMotor;
  PWMSim m_simRightFrontAngleMotor;
  PWMSim m_simRightBackAngleMotor;
  CANcoderSimState m_rightBackEncoder;
  CANcoderSimState m_rightFrontEncoder;



  PWMSim m_simLeftFrontDriveMotor;
  PWMSim m_simLeftBackDriveMotor;
  PWMSim m_simLeftFrontAngleMotor;
  PWMSim m_simLeftBackAngleMotor;
  CANcoderSimState m_leftBackEncoder;
  CANcoderSimState m_leftFrontEncoder;


  @BeforeEach // this method will run before each test
  void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed

    //TODO - add logic to pull channels from json files
    m_swerve = new SwerveSubsystem(); // create our intake
    m_simRightFrontDriveMotor = new PWMSim(4);
    m_simRightFrontAngleMotor = new PWMSim(5);
    m_rightFrontEncoder = new CANcoderSimState(new CoreCANcoder(10, "rio"));

    m_simRightBackDriveMotor = new PWMSim(2);
    m_simRightBackAngleMotor = new PWMSim(3);
    m_rightBackEncoder = new CANcoderSimState(new CoreCANcoder(9, "rio"));
  
    m_simLeftFrontDriveMotor = new PWMSim(6);
    m_simLeftFrontAngleMotor = new PWMSim(7);
    m_leftFrontEncoder = new CANcoderSimState(new CoreCANcoder(11, "rio"));

    m_simLeftBackDriveMotor = new PWMSim(0);
    m_simLeftBackAngleMotor = new PWMSim(1);
    m_leftBackEncoder = new CANcoderSimState(new CoreCANcoder(8, "rio"));
}

  @SuppressWarnings("PMD.SignatureDeclareThrowsException")
  @AfterEach // this method will run after each test
  void shutdown() throws Exception {
  }

  @Test
  void testSwerve() {
    assertNotNull(m_swerve);
  }
}
