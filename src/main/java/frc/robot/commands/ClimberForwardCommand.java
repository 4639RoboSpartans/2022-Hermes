package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberForwardCommand extends CommandBase{
    private ClimberSubsystem m_climber;

    public ClimberForwardCommand(ClimberSubsystem m_climber){
        this.m_climber = m_climber;
        addRequirements(m_climber);
    }

    @Override
    public void initialize(){
        m_climber.setClimber(0, 0);
    }

    @Override
    public void execute(){
        m_climber.setClimber(0.5, 0.5);
    }
    
    @Override
    public void end(boolean Interrupted){
        m_climber.stopClimber();
    }

    @Override
    public boolean isFinished(){
        return false;
    }

}
