package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class OutTakeCommand extends CommandBase{
    private IntakeSubsystem m_intake ;
    private HopperSubsystem m_hopper;
    private FeederSubsystem m_feeder ;
    public OutTakeCommand(IntakeSubsystem m_intake, HopperSubsystem m_hopper, FeederSubsystem m_feeder){
        this.m_intake=m_intake;
        this.m_hopper =m_hopper;
        this.m_feeder =m_feeder;
        addRequirements(m_intake, m_hopper, m_feeder);
    }
    @Override
    public void initialize(){
        m_intake.setIntake(0);
        m_hopper.setHopper(0);
        m_feeder.setFeeder(0);
    }
    @Override
    public void execute(){
        m_intake.setIntake(-0.4);
        m_hopper.setHopper(-0.4);
        m_feeder.setFeeder(-0.4);
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
