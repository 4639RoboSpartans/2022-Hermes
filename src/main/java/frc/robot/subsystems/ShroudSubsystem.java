package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShroudSubsystem extends SubsystemBase{
    private WPI_VictorSPX ShroudMotor = new WPI_VictorSPX(Constants.ShroudMotor);
    public ShroudSubsystem(){
        ShroudMotor.configFactoryDefault();
        ShroudMotor.setNeutralMode(NeutralMode.Brake);
    }
    public void setShroud(double speed){
        ShroudMotor.set(speed);
    }
    public void stopShroud(){
        ShroudMotor.set(0);
    }
}
