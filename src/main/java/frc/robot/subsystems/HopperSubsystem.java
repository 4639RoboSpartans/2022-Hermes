package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HopperSubsystem extends SubsystemBase {
    private WPI_VictorSPX hopperMotor = new WPI_VictorSPX(Constants.HopperMotor);
    public HopperSubsystem(){
        hopperMotor.configFactoryDefault();
        hopperMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void setHopper(double speed){
        hopperMotor.set(speed);
    }
    
    public void stopHopper(){
        hopperMotor.set(0);
    }
}
