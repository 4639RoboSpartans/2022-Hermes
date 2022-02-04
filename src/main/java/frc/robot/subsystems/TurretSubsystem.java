package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {
    private WPI_TalonFX turretMotor = new WPI_TalonFX(Constants.TurretMotor);
    public TurretSubsystem(){
        turretMotor.configFactoryDefault();
        turretMotor.setNeutralMode(NeutralMode.Coast);
    }
}
