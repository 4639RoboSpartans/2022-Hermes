package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase{
    private IntakeSubsystem m_intake = new IntakeSubsystem();
    private HopperSubsystem m_hopper = new HopperSubsystem();
    private FeederSubsystem m_feeder = new FeederSubsystem();
    private DigitalInput ballOccupied = new DigitalInput(Constants.feederSensor);
    public IntakeCommand(){
        addRequirements(m_intake, m_hopper,m_feeder);
    }
    @Override
    public void initialize(){
        m_intake.setIntake(0);
        m_hopper.setHopper(0);
        m_feeder.setFeeder(0);
    }
    @Override
    public void execute(){
        m_intake.setIntake(0.5);
        m_hopper.setHopper(0.5);
        if(ballOccupied.get()){
            m_feeder.setFeeder(0.2);
        }else{
            m_feeder.stop();
        }

    }
    @Override
    public void end(boolean Interrupted){
        m_intake.stopIntake();
        m_hopper.stopHopper();
        m_feeder.stop();
    }
    @Override
    public boolean isFinished(){
        return false;
    }
}
