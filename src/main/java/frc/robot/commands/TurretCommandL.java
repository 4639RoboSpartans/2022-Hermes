package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShroudSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TurretCommandL extends CommandBase{
    public TurretSubsystem m_turret;
    public ShroudSubsystem m_shroud;
    public TurretCommandL(TurretSubsystem m_turret){
        this.m_turret = m_turret;
        addRequirements(m_turret);
    }
    @Override
    public void initialize(){
        m_turret.setTurret(0);
    }
    @Override
    public void execute(){
        m_turret.setTurretVolt(5);
        // m_shroud.setShroud(-0.4);

    }
    @Override
    public void end(boolean en){
        m_turret.setTurret(0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
