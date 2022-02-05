package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberBackwardCommand extends CommandBase{
    private ClimberSubsystem m_climber = new ClimberSubsystem();

    public ClimberBackwardCommand(){
        addRequirements(m_climber);
    }

    @Override
    public void initialize(){
        m_climber.setClimber(0, 0);
    }

    @Override
    public void execute(){
        m_climber.setClimber(-0.2, -0.2);
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
