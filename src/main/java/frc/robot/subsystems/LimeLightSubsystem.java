package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLightSubsystem extends SubsystemBase {
    public NetworkTable LLTable = NetworkTableInstance.getDefault().getTable("limelight");
    public LimeLightSubsystem(){
        LLTable.getEntry("camMode").setNumber(0); //sets camera to vision processing mode
        LLTable.getEntry("pipeline").setNumber(0);//sets the pipeline to 0 which is default  
    }
    public boolean targetVisible(){
         if(LLTable.getEntry("tv").getDouble(0)==1){
            return true;
        }
        return false;
    }
    public double targetx(){
        if(targetVisible()){
            return LLTable.getEntry("tx").getDouble(0);
        }else{
            return -1;
        }
    }
    public double targety(){
        if(targetVisible()){
            return LLTable.getEntry("ty").getDouble(0);
        }else{
            return -1;
        }
    }
    
}
