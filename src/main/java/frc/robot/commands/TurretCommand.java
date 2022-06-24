package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.ShroudSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TurretCommand extends CommandBase{
    public TurretSubsystem m_turret;
    public ShroudSubsystem m_shroud;
    public OI m_oi;
    public TurretCommand(TurretSubsystem m_turret, OI m_oi){
        this.m_turret = m_turret;
        this.m_oi = m_oi;
        addRequirements(m_turret);
    }
    @Override
    public void initialize(){
        m_turret.setTurret(0);
    }
    @Override
    public void execute(){
    //     if(m_oi.getButton(0, Constants.Buttons.Y_BUTTON).get())
    //    m_turret.setTurretVolt(5);
    //    else
       m_turret.setTurretVolt(0);
        // m_shroud.setShroud(0.4);
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
