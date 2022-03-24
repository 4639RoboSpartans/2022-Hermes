package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase{
    private WPI_TalonFX ShooterMotor = new WPI_TalonFX(Constants.ShooterMotor);
    public ShooterSubsystem(){
        ShooterMotor.configFactoryDefault();
        ShooterMotor.setNeutralMode(NeutralMode.Coast);
        ShooterMotor.configVoltageCompSaturation(12);
        ShooterMotor.enableVoltageCompensation(true);
        ShooterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    }
    public void setShooter(double speed){
        ShooterMotor.set(-speed);
    }
    public void setShooterVolt(double volts){
        ShooterMotor.setVoltage(-volts);
    }
    public void stopShooter(){
        ShooterMotor.set(0);
    }
    public double getRate(){
        return ((-ShooterMotor.getSelectedSensorVelocity()));
    }
    public double getPosition(){
        return -ShooterMotor.getSelectedSensorPosition();
    } 
}
