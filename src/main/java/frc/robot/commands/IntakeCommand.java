package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase{
    private IntakeSubsystem m_intake = new IntakeSubsystem();
    public IntakeCommand(){
        addRequirements(m_intake);
    }
    @Override
    public void initialize(){
        m_intake.setIntake(0);
    }
    @Override
    public void execute(){
        m_intake.setIntake(0.5);
    }
    @Override
    public void end(boolean Interrupted){
        m_intake.stopIntake();
    }
    @Override
    public boolean isFinished(){
        return false;
    }
}
