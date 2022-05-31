package frc.robot.subsystems;

import javax.swing.text.DefaultStyledDocument.ElementSpec;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
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
        // ShooterMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true,35,40,1.0));
        // ShooterMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,30,35,0.5));
        ShooterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        
    }
    public void setShooter(double speed){
        if(!Constants.climbing){
        ShooterMotor.set(-speed);
        }else{
            ShooterMotor.set(0);
        }
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
    public void startClimbing(){
        Constants.climbing = true;
    }
    public void stopClimbing(){
        Constants.climbing = false;
    }
}
