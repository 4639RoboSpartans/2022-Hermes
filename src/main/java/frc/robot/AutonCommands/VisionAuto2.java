package frc.robot.AutonCommands;

import java.io.Console;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShroudSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class VisionAuto2 extends CommandBase {
    private LimeLightSubsystem LL;
    private TurretSubsystem m_turret;
    private ShroudSubsystem m_shroud;

    private ShooterSubsystem m_shooter;
    private FeederSubsystem m_feeder;
    private HopperSubsystem m_hopper;

    public double LLHeight = 23.5;
    public double TargetHeight = 104;// inches
    public double LLAngle = 33;

    double limelightMountAngleDegrees = 33.0;
    double limelightLensHeightInches = 23.5;
    double goalHeightInches = 104.0;
    PIDController PIDVTurret = new PIDController(0.035, 0.0002, 0);
    PIDController PIDVShroud = new PIDController(0.00265, 0.0035, 0.0000);// 0.0014, 0.0044,0.00

    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.59406, 0.1132, 0.040239);
    private BangBangController shooterBang = new BangBangController(5);

    private OI m_oi;
    Timer time;
    public VisionAuto2(LimeLightSubsystem LL, TurretSubsystem m_turret, ShroudSubsystem m_shroud,
            ShooterSubsystem m_shooter, OI m_oi, FeederSubsystem m_feeder, HopperSubsystem m_hopper) {
        this.LL = LL;
        this.m_turret = m_turret;
        this.m_shroud = m_shroud;
        this.m_shooter = m_shooter;
        this.m_feeder = m_feeder;
        this.m_hopper = m_hopper;
        this.m_oi = m_oi;
        PIDVTurret.setTolerance(0);
        PIDVShroud.setTolerance(5);
        time = new Timer();
        addRequirements(LL, m_turret, m_shroud, m_shooter, m_feeder, m_hopper);
    }

    @Override
    public void initialize() {
        m_turret.setTurret(0);
        m_shroud.setShroud(0);
        m_shooter.setShooter(0);
        m_feeder.setFeeder(0);
        m_hopper.setHopper(0);
        time.start();
    }

    @Override
    public void execute() {
        double targetOffsetAngle_Vertical = LL.LLTable.getEntry("ty").getDouble(0.0);
        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
        SmartDashboard.putNumber("Distance From Target", distanceFromLimelightToGoalInches);
        double desiredPosition = -0.003*(Math.pow(distanceFromLimelightToGoalInches,2))+2.1423*distanceFromLimelightToGoalInches-136.6;
        // -0.003*(Math.pow(distanceFromLimelightToGoalInches,2))+2.1423*distanceFromLimelightToGoalInches-116.6;
        //-0.0031 * Math.pow(distanceFromLimelightToGoalInches, 2)
        // + 2.4152 * distanceFromLimelightToGoalInches - 180;

        double desiredSpeed = 4.0544*distanceFromLimelightToGoalInches+5700;
        // 4.0544*distanceFromLimelightToGoalInches+5595.3;
        // -0.0045 * Math.pow(distanceFromLimelightToGoalInches, 2)
                // + 8.6897 * distanceFromLimelightToGoalInches + 4550;
      
            if (LL.LLTable.getEntry("tx").getDouble(0) != -1) {
                SmartDashboard.putNumber("VAlue", -PIDVTurret.calculate(LL.LLTable.getEntry("tx").getDouble(0), 0));
                m_turret.setTurret(-PIDVTurret.calculate(LL.LLTable.getEntry("tx").getDouble(0), 0));
                m_shroud.setShroud(PIDVShroud.calculate(m_shroud.getShroudPosition(), desiredPosition));
                m_shooter.setShooter(shooterBang.calculate(m_shooter.getRate(), desiredSpeed) *8000
                        + 0.0006 * feedforward.calculate(desiredSpeed));
            } else {
                SmartDashboard.putBoolean("Shooter", false);
            }
            if (m_shooter.getRate() > 1.15 * desiredSpeed) {
               
            m_hopper.setHopper(0.7);
            m_feeder.setFeeder(0.6);
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
        m_shooter.stopShooter();
    
    }

    @Override
    public boolean isFinished() {
        if(time.get()>2){
            time.stop();
            return true;
        }
        return false;
    }

}
