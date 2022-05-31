package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
    private WPI_VictorSPX leftClimberMotor = new WPI_VictorSPX(Constants.LClimberMotor);
    private WPI_VictorSPX rightClimberMotor = new WPI_VictorSPX(Constants.RClimberMotor);

    public Solenoid climberSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 7);

    public ClimberSubsystem(){
        leftClimberMotor.configFactoryDefault();
        rightClimberMotor.configFactoryDefault();

        leftClimberMotor.setNeutralMode(NeutralMode.Brake);
        rightClimberMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void setClimber(double leftSpeed, double rightSpeed){
        leftClimberMotor.set(leftSpeed);
        rightClimberMotor.set(rightSpeed);
    }
    public void setLeftClimber(double leftSpeed){
        leftClimberMotor.set(leftSpeed);
    }
    public void setRightClimber(double rightSpeed){
        rightClimberMotor.set(rightSpeed);
    }

    public void extendPistons(){
        climberSolenoid.set(true);
    }
    public void retractPistons(){
        climberSolenoid.set(false);
    }

    public void stopClimber(){
        leftClimberMotor.set(0);
        rightClimberMotor.set(0);
    }
}
