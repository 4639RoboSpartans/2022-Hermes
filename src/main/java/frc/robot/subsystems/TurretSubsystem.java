package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {
    public WPI_TalonFX turretMotor = new WPI_TalonFX(Constants.TurretMotor);
    public PIDController turretPID = new PIDController(0.035, 0.0002, 0);
    public TurretSubsystem(){
        turretMotor.configFactoryDefault();
        turretMotor.setNeutralMode(NeutralMode.Brake);
        turretMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    }
    //10 tooth motor to 140 tooth
    //2048 units per revolution
    //9:1 for speed
    public void setTurret(double speed){
        // if(getTurretRot()<=0&&speed<0){
        //     turretMotor.set(0);
        // }else if(getTurretRot()>=4&&speed>0){
        //     turretMotor.set(0);
        // }else{
            turretMotor.set(speed);
        // }
        
    }
    public void setTurretVolt(double volt){
        turretMotor.setVoltage(volt);
    }
    public void stopTurret(){
        turretMotor.set(0);
    }
    //range of 49,000
    public double getTurretRot(){
        double deg = turretMotor.getSelectedSensorPosition();
        deg /=2048;
        deg*=9;
        deg/=14;
        
        return deg;
    }
    public double getTurretRotRate(){
        return turretMotor.getSelectedSensorVelocity();
    }
    public void TurretClimbing(){
        setTurret(-turretPID.calculate(getTurretRot(),0));
    }
}
