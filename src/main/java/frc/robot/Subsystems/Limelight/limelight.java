package frc.robot.Subsystems.Limelight;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class limelight {
    private NetworkTable table;
    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry ta;
    NetworkTableEntry tid;
    private double x;
    private double y;
    private double area;
    private int tagId;

    public limelight(String cameraName) {
        table = NetworkTableInstance.getDefault().getTable(cameraName);
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        tid = table.getEntry("tid");
        
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getArea() {
        return area;
    }
    
    public int getTagId() {
        return tagId;
    }

    public void periodic() {
        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        area = ta.getDouble(0.0);
        tagId = (int)tid.getInteger(0);

    }
    
}
