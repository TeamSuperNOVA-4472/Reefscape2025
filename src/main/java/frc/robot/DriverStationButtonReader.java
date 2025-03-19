package frc.robot;

import com.fazecast.jSerialComm.SerialPort;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.io.BufferedReader;
import java.io.InputStreamReader;

public class DriverStationButtonReader {

    private NetworkTable table;
    private SerialPort serialPort;
    private BufferedReader reader;

    public DriverStationButtonReader() {
        // Connect to the NetworkTable instance
        table = NetworkTableInstance.getDefault().getTable("Arduino");
    }

    // Method to start serial communication and set up the reader
    public void initializeSerial() {
        serialPort = SerialPort.getCommPorts()[0];  // Select the first available port (you may need to change this)
        serialPort.openPort();
        reader = new BufferedReader(new InputStreamReader(serialPort.getInputStream()));
    }

    // Method to parse serial data and update NetworkTable entries
    public void updateButtonStates() {
        try {
            String data = reader.readLine();  // Read the line of button states
            String[] buttonStates = data.split(",");  // Split the comma-separated values
            
            for (int i = 0; i < buttonStates.length; i++) {
                // Update the NetworkTable with the button states (convert to integer or boolean as needed)
                table.getEntry("button" + i).setBoolean(Integer.parseInt(buttonStates[i]) == 1);
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    // Main method
    public static void main(String[] args) {
        DriverStationButtonReader reader = new DriverStationButtonReader();
        reader.initializeSerial();  // Initialize serial connection

        try {
            NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
            networkTableInstance.startClient4("client");
            networkTableInstance.setServerTeam(4472);
            networkTableInstance.startDSClient();

            while (true) {
                reader.updateButtonStates();  // Read serial data and update button states
                Thread.sleep(100);  // Delay for a short period to avoid overloading the CPU
            }
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
