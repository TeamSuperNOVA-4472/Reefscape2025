package frc.robot.objectmodels;

import com.fazecast.jSerialComm.SerialPort;
import edu.wpi.first.wpilibj.Joystick;

public class ArduinoButtonSimulator {
    private SerialPort arduinoPort;
    private boolean[] buttonStates = new boolean[10];  // Array to store states of 10 buttons
    private Joystick joystick;  // Use WPILib Joystick class to simulate joystick

    public ArduinoButtonSimulator() {
        joystick = new Joystick(0);  // Use the joystick on port 0
    }

    public void initialize() {
        // Get available serial ports
        SerialPort[] ports = SerialPort.getCommPorts();

        // Find the port where your Arduino is connected
        for (SerialPort port : ports) {
            System.out.println("Available port: " + port.getSystemPortName());
            if (port.getSystemPortName().equals("/dev/ttyUSB0")) {  // Adjust port name for your system
                arduinoPort = port;
                break;
            }
        }

        if (arduinoPort != null) {
            arduinoPort.openPort();  // Open the port
            arduinoPort.setBaudRate(9600);  // Baud rate must match the Arduino sketch
            arduinoPort.setComPortTimeouts(SerialPort.TIMEOUT_READ_BLOCKING, 0, 0);
        } else {
            System.out.println("Arduino not found on any available serial ports.");
        }
    }

    public void updateButtons() {
        if (arduinoPort != null) {
            byte[] readBuffer = new byte[10];  // Adjust buffer size for 10 buttons
            int numBytes = arduinoPort.readBytes(readBuffer, readBuffer.length);

            if (numBytes > 0) {
                String data = new String(readBuffer, 0, numBytes).trim();  // Read data
                String[] values = data.split(",");

                if (values.length == 10) {  // Expect 10 button states
                    try {
                        // Update button states
                        for (int i = 0; i < 10; i++) {
                            buttonStates[i] = Integer.parseInt(values[i]) == 1;  // Normal logic: 1 = pressed
                        }
                    } catch (NumberFormatException e) {
                        e.printStackTrace();
                    }
                }
            }
        }
    }

    // This method simulates joystick button presses based on buttonStates
    public void simulateJoystick() {
        for (int i = 0; i < 10; i++) {
            // You could use buttonStates here to trigger actions in your robot
            boolean buttonPressed = buttonStates[i];
            System.out.println("Button " + (i + 1) + " state: " + (buttonPressed ? "Pressed" : "Released"));
        }
    }

    public boolean getButton(int i)
    {
        return buttonStates[i];
    }

    public void close() {
        if (arduinoPort != null) {
            arduinoPort.closePort();
        }
    }
}
