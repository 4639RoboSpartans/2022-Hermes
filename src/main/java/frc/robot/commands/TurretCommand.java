package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShroudSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TurretCommand extends CommandBase{
    public TurretSubsystem m_turret;
    public ShroudSubsystem m_shroud;
    public TurretCommand(TurretSubsystem m_turret, ShroudSubsystem m_shroud){
        this.m_turret = m_turret;
        this.m_shroud=m_shroud;
        addRequirements(m_turret, m_shroud);
    }
    @Override
    public void initialize(){
        m_turret.setTurret(0);
        m_shroud.setShroud(0);
    }
    @Override
    public void execute(){
       // m_turret.setTurret(0.2);
        m_shroud.setShroud(0.4);
    }
    @Override
    public void end(boolean en){
        m_turret.setTurret(0);
        m_shroud.setShroud(0);
    }

}
