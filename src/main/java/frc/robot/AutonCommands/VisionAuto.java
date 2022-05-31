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
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShroudSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class VisionAuto extends CommandBase {
    private LimeLightSubsystem LL;
    private TurretSubsystem m_turret;
    private ShroudSubsystem m_shroud;

    private ShooterSubsystem m_shooter;
    private FeederSubsystem m_feeder;
    private HopperSubsystem m_hopper;

    private IntakeSubsystem m_intake;
    private boolean runningIntake = false;

    public double LLHeight = 35;
    public double TargetHeight = 104;// inches
    public double LLAngle = 25;

    double limelightMountAngleDegrees = 25.0;
    double limelightLensHeightInches = 35;
    double goalHeightInches = 104.0;

    double tim = 0;
    PIDController PIDVTurret = new PIDController(0.4, 0.000, 0);//.035
    PIDController PIDVShroud = new PIDController(0.005 , 0.012, 0.0000);// 0.0015, 0.012

    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.7591 , 0.13571, 0.035856);//new
    // private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.60213, 0.12494, 0.043843);
    private BangBangController shooterBang = new BangBangController(5);
    // private PIDController shooterPID = new PIDController(1.6, 0.015, 0);//1.511
       

    private OI m_oi;
    Timer time;



    public VisionAuto(LimeLightSubsystem LL, TurretSubsystem m_turret, ShroudSubsystem m_shroud,
            ShooterSubsystem m_shooter, OI m_oi, FeederSubsystem m_feeder, HopperSubsystem m_hopper, IntakeSubsystem m_intake, boolean runningIntake, double tim) {
        this.LL = LL;
        this.tim = tim;
        this.m_turret = m_turret;
        this.m_shroud = m_shroud;
        this.m_shooter = m_shooter;
        this.m_feeder = m_feeder;
        this.m_hopper = m_hopper;
        this.m_oi = m_oi;
        this.m_intake = m_intake;
        this.runningIntake = runningIntake;
        PIDVTurret.setTolerance(0);
        PIDVShroud.setTolerance(5);
        time = new Timer();
        addRequirements(LL, m_turret, m_shroud, m_shooter, m_feeder, m_hopper, m_intake);
    }

    @Override
    public void initialize() {
        m_turret.setTurret(0);
        m_shroud.setShroud(0);
        m_shooter.setShooter(0);
        m_feeder.setFeeder(0);
        m_hopper.setHopper(0);
        m_intake.setIntake(0);
        if(runningIntake){
            m_intake.extendPistons();
        }
        time.start();
    }

    @Override
    public void execute() {
        double targetOffsetAngle_Vertical=0;
        double angleToGoalDegrees=0;
        double angleToGoalRadians=0 ;
        double distanceFromLimelightToGoalInches=0;
        double desiredPosition=0;
        double desiredSpeed=0;
        double HorizontalOffset=0;
        
            HorizontalOffset = LL.LLTable.getEntry("tx").getDouble(0);
            targetOffsetAngle_Vertical = LL.LLTable.getEntry("ty").getDouble(0.0);
        angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        angleToGoalRadians= angleToGoalDegrees * (3.14159 / 180.0);
        // if(Math.abs(LL.LLTable.getEntry("tx").getDouble(0))>5){
        distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
        // }
        SmartDashboard.putNumber("Distance From Target", distanceFromLimelightToGoalInches);
        desiredPosition= -0.0016*Math.pow(distanceFromLimelightToGoalInches,2) + 1.4015*distanceFromLimelightToGoalInches - 168.25;
        // desiredPosition= -0.0016*Math.pow(distanceFromLimelightToGoalInches,2) + 1.4015*distanceFromLimelightToGoalInches - 130.25;
        // 4.0544*distanceFromLimelightToGoalInches+5595.3;
        // -0.0045 * Math.pow(distanceFromLimelightToGoalInches, 2)
                // + 8.6897 * distanceFromLimelightToGoalInches + 4550;
      
            if (LL.LLTable.getEntry("tx").getDouble(0) != -1) {
                SmartDashboard.putNumber("VAlue", -PIDVTurret.calculate(LL.LLTable.getEntry("tx").getDouble(0), 0));
                
                
                m_turret.setTurretVolt(-PIDVTurret.calculate(LL.LLTable.getEntry("tx").getDouble(0), -5));
                // else
                // m_turret.setTurret(0);
                // desiredPosition=160;
                // desiredSpeed=42;
                // desiredPosition = 50;
                double sped;
                if(distanceFromLimelightToGoalInches>430){
                    desiredSpeed= Math.min(38,0.0001*Math.pow(distanceFromLimelightToGoalInches, 2) - 0.0094*distanceFromLimelightToGoalInches+ 30);
                    sped = shooterBang.calculate(m_shooter.getRate(), desiredSpeed)+ (feedforward.calculate(desiredSpeed)*1./12);
                }
                else if(distanceFromLimelightToGoalInches>340){
                    desiredSpeed= Math.min(38,0.0001*Math.pow(distanceFromLimelightToGoalInches, 2) - 0.0094*distanceFromLimelightToGoalInches+ 31);
                    sped = shooterBang.calculate(m_shooter.getRate(), desiredSpeed)+ (feedforward.calculate(desiredSpeed)*1./12);
                }
                else if(distanceFromLimelightToGoalInches>290){
                    desiredSpeed= Math.min(38,0.0001*Math.pow(distanceFromLimelightToGoalInches, 2) - 0.0094*distanceFromLimelightToGoalInches+ 32);
                    sped = shooterBang.calculate(m_shooter.getRate(), desiredSpeed)+ (feedforward.calculate(desiredSpeed)*1./12);
                }else if(distanceFromLimelightToGoalInches<162){
                    desiredSpeed= Math.min(38,0.0001*Math.pow(distanceFromLimelightToGoalInches, 2) - 0.0094*distanceFromLimelightToGoalInches+ 23);
                    sped = shooterBang.calculate(m_shooter.getRate(), desiredSpeed+5)+ (feedforward.calculate(desiredSpeed+5)*1.00/12);
                }else{
                    
                desiredSpeed= Math.min(38,0.0001*Math.pow(distanceFromLimelightToGoalInches, 2) - 0.0094*distanceFromLimelightToGoalInches+ 31);
                sped = shooterBang.calculate(m_shooter.getRate(), desiredSpeed)+ (feedforward.calculate(desiredSpeed)*1.00/12);

                // if((Constants.leftRate+Constants.rightRate)/2<-0.05){
                //     m_shroud.setShroud(PIDVShroud.calculate(m_shroud.getShroudPosition(), Math.max(0,desiredPosition+(100*((Constants.leftRate+Constants.rightRate)/2)))));
                //     Constants.moveshoot = true;
                // }else if((Constants.leftRate+Constants.rightRate)/2>0.05){
                //     Constants.moveshoot = true;
                //     m_shroud.setShroud(PIDVShroud.calculate(m_shroud.getShroudPosition(), Math.max(0,desiredPosition-(100*((Constants.leftRate+Constants.rightRate)/2)))));
                // }else{
                //     Constants.moveshoot = false;
                   
                // }
               
                // desiredSpeed = (desiredSpeed/2048)*10;

                // sped=0;
            }
                if(runningIntake){
                 sped*=0.92;
                m_shroud.setShroud(PIDVShroud.calculate(m_shroud.getShroudPosition(), Math.max(0,desiredPosition-20)));
                }else{
                    m_shroud.setShroud(PIDVShroud.calculate(m_shroud.getShroudPosition(), Math.max(0,desiredPosition)));
                }
                m_shooter.setShooter(sped);
                // m_shooter.setShooter(shooterBang.calculate(m_shooter.getRate(), 4550)
                // + feedforward.calculate(desiredSpeed)*0.00043);
                
            }
            if(runningIntake){
                m_intake.setIntake(0.7);
            }else{
                m_intake.setIntake(0);
            }
            if ((m_shooter.getRate()/2048)*10 > desiredSpeed) {
                m_hopper.setHopper(0.7);
                m_feeder.setFeeder(0.6);
            } else{
                m_hopper.setHopper(0);
                m_feeder.setFeeder(0);
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
        m_hopper.stopHopper();
        m_feeder.stop();
    
    }

    @Override
    public boolean isFinished() {
        if(time.get()>tim){
            time.stop();
            return true;
        }
        return false;
    }

}
