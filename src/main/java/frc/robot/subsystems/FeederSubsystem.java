package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FeederSubsystem extends SubsystemBase{
    private WPI_VictorSPX feederMotor = new WPI_VictorSPX(Constants.FeedMotor);
}
