package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShroudSubsystem extends SubsystemBase{
    private WPI_VictorSPX ShroudMotor = new WPI_VictorSPX(Constants.ShroudMotor);
    private Encoder ShroudEncoder = new Encoder(2,3);
    public ShroudSubsystem(){
        ShroudMotor.configFactoryDefault();
        ShroudMotor.setNeutralMode(NeutralMode.Brake);
        ShroudEncoder.setDistancePerPulse(360.0/2048);
    }
    public void setShroud(double speed){
        ShroudMotor.set(speed);
    }
    public void stopShroud(){
        ShroudMotor.set(0);
    }
    public double getShroudPosition(){
        return ShroudEncoder.getDistance();
    }
    public double getShroudRate(){
        return ShroudEncoder.getRate();
    }
}
