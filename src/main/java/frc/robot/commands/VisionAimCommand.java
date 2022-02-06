package frc.robot.commands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.ShroudSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class VisionAimCommand extends CommandBase {
    private LimeLightSubsystem LL = new LimeLightSubsystem();
    private TurretSubsystem m_turret = new TurretSubsystem();
    private ShroudSubsystem m_shroud = new ShroudSubsystem();
    PIDController PIDVTurret = new PIDController(0, 0, 0);
    PIDController PIDVShroud = new PIDController(0, 0, 0);
    PIDController PIDETurret = new PIDController(0, 0, 0);
    PIDController PIDEShroud = new PIDController(0, 0, 0);
    public VisionAimCommand(){
        PIDVTurret.setTolerance(10);
        PIDVShroud.setTolerance(10);
        PIDETurret.setTolerance(10);
        PIDEShroud.setTolerance(10);
        addRequirements(LL, m_turret, m_shroud);
    }
    @Override
    public void initialize(){
        m_turret.setTurret(PIDETurret.calculate(m_turret.getTurretRot(),0));
        m_shroud.setShroud(PIDEShroud.calculate(m_shroud.getShroudPosition(), 0));
    }
    @Override
    public void execute(){
        if(LL.targetVisible()){
            m_turret.setTurret(PIDVTurret.calculate(LL.targetx(), 0));
            m_shroud.setShroud(PIDVShroud.calculate(LL.targety(),0));
        }else{
            //put tracking code here
        }
    }
    @Override
    public void end(boolean interrupted){
        m_turret.setTurret(PIDETurret.calculate(m_turret.getTurretRot(),0));
        m_shroud.setShroud(PIDEShroud.calculate(m_shroud.getShroudPosition(), 0));
    }
    @Override
    public boolean isFinished(){
        return false;
    }

}
