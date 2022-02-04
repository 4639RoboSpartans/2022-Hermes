package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.fasterxml.jackson.annotation.Nulls;

import org.opencv.features2d.Feature2D;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FeederSubsystem extends SubsystemBase{
    private WPI_VictorSPX feederMotor = new WPI_VictorSPX(Constants.FeedMotor);
    public FeederSubsystem(){
        feederMotor.configFactoryDefault();
        feederMotor.setNeutralMode(NeutralMode.Coast);
    }
    public void setFeeder(double speed){
        feederMotor.set(speed);
    }
    public void stop(){
        feederMotor.set(0);
    }
}
