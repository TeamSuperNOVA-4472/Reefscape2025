package frc.robot.objectmodels;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import com.fazecast.jSerialComm.SerialPort;

public class ArduinoButtonSimulator {
    private SerialPort arduinoPort;
    private boolean[] buttonStates = new boolean[10];  // Array to store states of 10 buttons

    public ArduinoButtonSimulator() {
        // Initialization logic, no joystick needed for this scenario
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
            byte[] readBuffer = new byte[64];  // Buffer size for serial data
            int numBytes = arduinoPort.readBytes(readBuffer, readBuffer.length);

            if (numBytes > 0) {
                String data = new String(readBuffer, 0, numBytes).trim();  // Read the data
                String[] values = data.split(",");

                if (values.length == 10) {  // Expect 10 button states
                    try {
                        // Update button states
                        for (int i = 0; i < 10; i++) {
                            buttonStates[i] = Integer.parseInt(values[i]) == 1;  // Normal logic: 1 = pressed
                        }

                        // Now, update NetworkTables with the button states
                        NetworkTableInstance inst = NetworkTableInstance.getDefault();
                        NetworkTable table = inst.getTable("Arduino");
                        for (int i = 0; i < 10; i++) {
                            table.getEntry("button" + i).setDouble(buttonStates[i] ? 1 : 0);
                        }

                    } catch (NumberFormatException e) {
                        e.printStackTrace();
                    }
                }
            }
        }
    }

    public void close() {
        if (arduinoPort != null) {
            arduinoPort.closePort();
        }
    }
}
