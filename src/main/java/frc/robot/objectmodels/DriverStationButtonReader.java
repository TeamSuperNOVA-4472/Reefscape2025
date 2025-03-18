package frc.robot.objectmodels;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class DriverStationButtonReader {
    private NetworkTable table;

    public DriverStationButtonReader() {
        // Connect to the NetworkTable instance
        table = NetworkTableInstance.getDefault().getTable("Arduino");
    }

    // Method to check if button 0 is pressed
    public boolean isButton0Pressed() {
        return table.getEntry("button0").getDouble(0) == 1; // Returns true if button 0 is pressed
    }

    // Method to check if button 1 is pressed
    public boolean isButton1Pressed() {
        return table.getEntry("button1").getDouble(0) == 1; // Returns true if button 1 is pressed
    }

    // Method to check if button 2 is pressed
    public boolean isButton2Pressed() {
        return table.getEntry("button2").getDouble(0) == 1; // Returns true if button 2 is pressed
    }

    // Method to check if button 3 is pressed
    public boolean isButton3Pressed() {
        return table.getEntry("button3").getDouble(0) == 1; // Returns true if button 3 is pressed
    }

    // Similarly for other buttons
    public boolean isButton4Pressed() {
        return table.getEntry("button4").getDouble(0) == 1;
    }

    public boolean isButton5Pressed() {
        return table.getEntry("button5").getDouble(0) == 1;
    }

    public boolean isButton6Pressed() {
        return table.getEntry("button6").getDouble(0) == 1;
    }

    public boolean isButton7Pressed() {
        return table.getEntry("button7").getDouble(0) == 1;
    }

    public boolean isButton8Pressed() {
        return table.getEntry("button8").getDouble(0) == 1;
    }

    public boolean isButton9Pressed() {
        return table.getEntry("button9").getDouble(0) == 1;
    }
}
