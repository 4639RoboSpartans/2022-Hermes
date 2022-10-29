package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.MotorCommutation;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.fasterxml.jackson.databind.AnnotationIntrospector.ReferenceProperty.Type;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {
    public CANSparkMax turretMotor = new CANSparkMax(Constants.TurretMotor, MotorType.kBrushless);
    public PIDController turretPID = new PIDController(
        // 0.05, 0.0002, 0
        .10, .0004, 0
    );
    public RelativeEncoder m_enc = turretMotor.getAlternateEncoder(4096);
    public TurretSubsystem(){
        
    }
    //10 tooth motor to 140 tooth
    //2048 units per revolution
    //9:1 for speed
    public void setTurret(double speed){
        // if(getTurretRot()<=4&&speed<0){
        //     turretMotor.setVoltage(0);
        // }else if(getTurretRot()>=4&&speed>0){
        //     turretMotor.setVoltage(0);
        // }else{
            turretMotor.set(speed);
        // }
        
    }
    public void setTurretVolt(double volt){
        // if(getTurretRot()<-6){
        //     turretMotor.setVoltage(0);
        // }else if(getTurretRot()>7){
        //     turretMotor.setVoltage(0);
        // }else
        {
            turretMotor.setVoltage(volt);
        }
    }
    public void stopTurret(){
        turretMotor.set(0);
    }
    //range of 49,000
    public double getTurretRot(){
        double deg = m_enc.getPosition();
        deg /=2048;
        deg*=9;
        deg/=14;
        
        return deg;
    }
    public double getTurretRotRate(){
        return m_enc.getPosition();
    }
    public void TurretClimbing(){
        setTurret(-turretPID.calculate(getTurretRot(),0));
    }
}
