package frc.robot.AutonCommands;

import java.io.Console;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.ShroudSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class VisionAuto extends CommandBase {
    private LimeLightSubsystem LL;
    private TurretSubsystem m_turret;
    private ShroudSubsystem m_shroud;
    public double LLHeight = 23.5;
    public double TargetHeight = 104;// inches
    public double LLAngle = 33;
    PIDController PIDVTurret = new PIDController(0.03, 0, 0);
    PIDController PIDVShroud = new PIDController(0.0014, 0.0004, 0.0000);// 0.0014, 0.0044,0.00
    Timer time;
    public VisionAuto(LimeLightSubsystem LL, TurretSubsystem m_turret, ShroudSubsystem m_shroud) {
        this.LL = LL;
        this.m_turret = m_turret;
        this.m_shroud = m_shroud;
        PIDVTurret.setTolerance(0.5);
        PIDVShroud.setTolerance(7);
        time = new Timer();
        addRequirements(LL, m_turret, m_shroud);
    }

    @Override
    public void initialize() {
        m_turret.setTurret(0);
        m_shroud.setShroud(0);
        time.start();
    }

    @Override
    public void execute() {
        NetworkTableEntry ty = LL.LLTable.getEntry("ty");
        double targetOffsetAngle_Vertical = ty.getDouble(0.0);
        double limelightMountAngleDegrees = 33.0;
        double limelightLensHeightInches = 23.5;
        double goalHeightInches = 104.0;
        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)
                / Math.tan(angleToGoalRadians);
        double desiredPosition = -0.0031 * Math.pow(distanceFromLimelightToGoalInches, 2)
                + 2.4152 * distanceFromLimelightToGoalInches - 180;
        if (LL.LLTable.getEntry("tx").getDouble(0) != -1) {
            SmartDashboard.putNumber("VAlue", -PIDVTurret.calculate(LL.LLTable.getEntry("tx").getDouble(0), 0));
            m_turret.setTurret(-PIDVTurret.calculate(LL.LLTable.getEntry("tx").getDouble(0), 0));
            m_shroud.setShroud(PIDVShroud.calculate(m_shroud.getShroudPosition(), desiredPosition));
        } else {
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
    public void end(boolean interrupted) {
        m_turret.stopTurret();
        m_shroud.stopShroud();
    }

    @Override
    public boolean isFinished() {
        if(time.get()>1){
            time.stop();
            return true;
        }
        return false;
    }


}
