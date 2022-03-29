package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
    private WPI_VictorSPX leftClimberMotor = new WPI_VictorSPX(Constants.LClimberMotor);
    private WPI_VictorSPX rightClimberMotor = new WPI_VictorSPX(Constants.RClimberMotor);

    public DoubleSolenoid leftClimberSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,2,3);
    public DoubleSolenoid rightClimberSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,4,5);

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
        leftClimberSolenoid.set(DoubleSolenoid.Value.kForward);
        rightClimberSolenoid.set(DoubleSolenoid.Value.kForward);
    }
    public void retractPistons(){
        leftClimberSolenoid.set(DoubleSolenoid.Value.kReverse);
        rightClimberSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void stopClimber(){
        leftClimberMotor.set(0);
        rightClimberMotor.set(0);
    }
}
