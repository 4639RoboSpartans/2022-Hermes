package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase{
    private IntakeSubsystem m_intake ;
    private HopperSubsystem m_hopper;
    private FeederSubsystem m_feeder ;
    private OI m_oi;
    public DigitalInput ballOccupied = new DigitalInput(Constants.feederSensor);
    public IntakeCommand(IntakeSubsystem m_intake, HopperSubsystem m_hopper, FeederSubsystem m_feeder, OI m_oi){
        this.m_intake=m_intake;
        this.m_hopper = m_hopper;
        this.m_feeder=m_feeder;
        this.m_oi = m_oi;
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
        if(m_oi.getAxis(1, Constants.Axes.RIGHT_TRIGGER)>0&&Constants.pushballs){
            m_intake.setIntake(0.7);
            m_hopper.setHopper(0.7);
            m_feeder.setFeeder(0.6);
        }
        else{
            m_intake.setIntake(0.7);
            m_hopper.setHopper(0.7);
            if(ballOccupied.get()){
                m_feeder.setFeeder(0.4);
            }else{
                m_feeder.stop();
            }
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
