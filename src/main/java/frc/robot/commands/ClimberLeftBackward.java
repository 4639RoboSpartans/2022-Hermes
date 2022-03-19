package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberLeftBackward extends CommandBase{
    private ClimberSubsystem m_climber;

    public ClimberLeftBackward(ClimberSubsystem m_climber){
        this.m_climber = m_climber;
        addRequirements(m_climber);
    }

    @Override
    public void initialize(){
        m_climber.setLeftClimber( 0);
    }

    @Override
    public void execute(){
        m_climber.setLeftClimber(-0.7);
    }
    
    @Override
    public void end(boolean Interrupted){
        m_climber.setLeftClimber(0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }

}
