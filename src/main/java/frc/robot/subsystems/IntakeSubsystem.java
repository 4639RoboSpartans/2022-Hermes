package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase{
    private WPI_VictorSPX IntakeMotor = new WPI_VictorSPX(Constants.IntakeMotor);
    private final Solenoid Piston = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.ForwardPiston);
    public IntakeSubsystem(){
        IntakeMotor.configFactoryDefault();
        IntakeMotor.setNeutralMode(NeutralMode.Coast);
    }
    public void setIntake(double speed){
        IntakeMotor.set(-speed);
    }
    public void stopIntake(){
        IntakeMotor.set(0);
    }
    public void extendPistons(){
        Piston.set(true);
    }
    public void retractPistons(){
        Piston.set(false);
    }
}
