package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem {
    private WPI_VictorSPX leftClimberMotor = new WPI_VictorSPX(Constants.LClimberMotor),
                          rightClimberMotor = new WPI_VictorSPX(Constants.RClimberMotor);

    public ClimberSubsystem(){
        leftClimberMotor.configFactoryDefault();
        rightClimberMotor.configFactoryDefault();

        leftClimberMotor.setNeutralMode(NeutralMode.Brake);
        rightClimberMotor.setNeutralMode(NeutralMode.Brake);
    }
    public setClimber(double leftSpeed, double rightSpeed){
        leftClimberMotor.set(leftSpeed);
        rightClimberMotor.set(rightSpeed);
    }
    public stopClimber(){
        leftClimberMotor.set(0);
        rightClimberMotor.set(0);
    }
}
