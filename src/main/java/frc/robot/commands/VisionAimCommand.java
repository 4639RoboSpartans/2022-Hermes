package frc.robot.commands;


import java.io.Console;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTableEntry;
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
    public double LLHeight = 23.5;
    public double TargetHeight=104;//inches
    public double LLAngle = 33;
    // private DriveSubsystem m_drive;
    //add feedforward mechanism!!!!!!!!
    PIDController PIDVTurret = new PIDController(0.03, 0, 0);
    PIDController PIDVShroud = new PIDController(0.0014, 0.0004, 0.0000);//0.0014, 0.0044,0.00
    public VisionAimCommand(LimeLightSubsystem LL, TurretSubsystem m_turret, ShroudSubsystem m_shroud){
        this.LL = LL;
        this.m_turret = m_turret;
        this.m_shroud = m_shroud;
        // this.m_drive = m_drive;
        PIDVTurret.setTolerance(0.5);
        PIDVShroud.setTolerance(7);
        addRequirements(LL, m_turret, m_shroud);
    }
    @Override
    public void initialize(){
        m_turret.setTurret(0);
        m_shroud.setShroud(0);
    }
    @Override
    public void execute(){
        NetworkTableEntry ty = LL.LLTable.getEntry("ty");
double targetOffsetAngle_Vertical = ty.getDouble(0.0);

// how many degrees back is your limelight rotated from perfectly vertical?
double limelightMountAngleDegrees = 33.0;

// distance from the center of the Limelight lens to the floor
double limelightLensHeightInches = 23.5;

// distance from the target to the floor
double goalHeightInches = 104.0;

double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

//calculate distance
double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians);
double desiredPosition  = -0.0031*Math.pow(distanceFromLimelightToGoalInches,2) + 2.4152*distanceFromLimelightToGoalInches - 180;

        //double dist =(TargetHeight-LLHeight)/Math.tan(Math.toRadians(Math.atan2(1,y)+LLAngle));
        //SmartDashboard.putNumber("Dist", distanceFromLimelightToGoalInches);
        if(LL.LLTable.getEntry("tx").getDouble(0)!=-1){
            SmartDashboard.putNumber("VAlue", -PIDVTurret.calculate(LL.LLTable.getEntry("tx").getDouble(0), 0));
            m_turret.setTurret(-PIDVTurret.calculate(LL.LLTable.getEntry("tx").getDouble(0), 0));
            m_shroud.setShroud(PIDVShroud.calculate(m_shroud.getShroudPosition(), desiredPosition));
            
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
