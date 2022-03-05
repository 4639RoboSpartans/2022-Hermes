package frc.robot.commands;


import java.io.Console;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.ShroudSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class VisionAimCommand extends CommandBase {
    private LimeLightSubsystem LL;
    private TurretSubsystem m_turret;
    private ShroudSubsystem m_shroud;    
    private double LLHeight = 23.5;
    private double TargetHeight=104;//inches
    private double LLAngle = 33;
    private DriveSubsystem m_drive;
    //add feedforward mechanism!!!!!!!!
    PIDController PIDVTurret = new PIDController(0.05, 0, 0);
    PIDController PIDVShroud = new PIDController(0.0012, 0.0017, 0);
    public VisionAimCommand(LimeLightSubsystem LL, TurretSubsystem m_turret, ShroudSubsystem m_shroud, DriveSubsystem m_drive){
        this.LL = LL;
        this.m_turret = m_turret;
        this.m_shroud = m_shroud;
        this.m_drive = m_drive;
        PIDVTurret.setTolerance(1);
        PIDVShroud.setTolerance(2);
        addRequirements(LL, m_turret, m_shroud, m_drive);
    }
    @Override
    public void initialize(){
        m_turret.setTurret(0);
        m_shroud.setShroud(0);
    }
    @Override
    public void execute(){
        SmartDashboard.putNumber("Dist", DistanceToTarget());
        if(LL.LLTable.getEntry("tx").getDouble(0)!=-1){
            SmartDashboard.putNumber("VAlue", -PIDVTurret.calculate(LL.LLTable.getEntry("tx").getDouble(0), 0));
            m_turret.setTurret(-PIDVTurret.calculate(LL.LLTable.getEntry("tx").getDouble(0), 0));
            m_shroud.setShroud(PIDVShroud.calculate(m_shroud.getShroudPosition(), 200));
        }else{
            //put tracking code here
            SmartDashboard.putBoolean("Shooter", false);
        }
    }
    public boolean targetVisible() {
        if (LL.LLTable.getEntry("tv").getDouble(0) == 1) {
            return true;
        }
        return false;
    }
    @Override
    public void end(boolean interrupted){
        m_turret.stopTurret();
        m_shroud.stopShroud();
        // m_turret.setTurret(PIDETurret.calculate(m_turret.getTurretRot(),0));
        // m_shroud.setShroud(PIDEShroud.calculate(m_shroud.getShroudPosition(), 0));
    }
    @Override
    public boolean isFinished(){
        return false;
    }
    public double DistanceToTarget() {
        return (TargetHeight-LLHeight)/Math.tan(Math.toRadians(getAngleY()+LLAngle));
    }

    public double getAngleX() {
        double nx = (1 / 160) * (LL.LLTable.getEntry("tx").getDouble(0) - 159.5);
        double vpw = 2.0 * Math.tan(Math.toRadians(54.0 / 2));
        double x = (vpw / 2) * nx;
        return Math.atan2(1, x);
    }

    public double getAngleY() {
        double ny = (1 / 160) * (119.5 - LL.LLTable.getEntry("ty").getDouble(0));
        double vph = 2.0 * Math.tan(Math.toRadians(41.0 / 2));
        double y = (vph / 2) * ny;
        return Math.atan2(1,y);    
    }

}
