package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class TurretCommandR extends CommandBase{
    public TurretSubsystem m_turret;
    public TurretCommandR(TurretSubsystem m_turret){
        this.m_turret = m_turret;
        addRequirements(m_turret);
    }
    @Override
    public void initialize(){
        m_turret.setTurret(0);
    }
    @Override
    public void execute(){
        m_turret.setTurret(-0.9);
    }
    @Override
    public void end(boolean en){
        m_turret.setTurret(0);
    }

}
