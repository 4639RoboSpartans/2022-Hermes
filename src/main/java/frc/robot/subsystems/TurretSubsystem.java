package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {
    private WPI_TalonFX turretMotor = new WPI_TalonFX(Constants.TurretMotor);
    public TurretSubsystem(){
        turretMotor.configFactoryDefault();
        turretMotor.setNeutralMode(NeutralMode.Brake);

        turretMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    }
    public void setTurret(double speed){
        if(getTurretRot()<=500&&speed<0){
            turretMotor.set(0);
        }else if(getTurretRot()>=42000&&speed>0){
            turretMotor.set(0);
        }else{
            turretMotor.set(speed);
        }
        
    }
    public void stopTurret(){
        turretMotor.set(0);
    }
    //range of 49,000
    public double getTurretRot(){
        return turretMotor.getSelectedSensorPosition();
    }
    public double getTurretRotRate(){
        return turretMotor.getSelectedSensorVelocity();
    }
}
