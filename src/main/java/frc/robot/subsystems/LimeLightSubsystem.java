package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLightSubsystem extends SubsystemBase {
    private NetworkTable LLTable = NetworkTableInstance.getDefault().getTable("limelight");
    private double LLHeight = 23.5;
    private double TargetHeight=104;//inches
    private double LLAngle = 33;
    public LimeLightSubsystem() {
        LLTable.getEntry("camMode").setNumber(0); // sets camera to vision processing mode
        LLTable.getEntry("pipeline").setNumber(0);// sets the pipeline to 0 which is default
    }

    public boolean targetVisible() {
        if (LLTable.getEntry("tv").getDouble(0) == 1) {
            return true;
        }
        return false;
    }

    public double targetx() {
        if (targetVisible()) {
            return LLTable.getEntry("tx").getDouble(0);
        } else {
            return -1;
        }
    }

    public double targety() {
        if (targetVisible()) {
            return LLTable.getEntry("ty").getDouble(0);
        } else {
            return -1;
        }
    }

    public double DistanceToTarget() {
        return (TargetHeight-LLHeight)/Math.tan(Math.toRadians(getAngleY()+LLAngle));
    }

    public double getAngleX() {
        double nx = (1 / 160) * (targetx() - 159.5);
        double vpw = 2.0 * Math.tan(Math.toRadians(54.0 / 2));
        double x = (vpw / 2) * nx;
        return Math.atan2(1, x);
    }

    public double getAngleY() {
        double ny = (1 / 160) * (119.5 - targety());
        double vph = 2.0 * Math.tan(Math.toRadians(41.0 / 2));
        double y = (vph / 2) * ny;
        return Math.atan2(1,y);    
    }

}
