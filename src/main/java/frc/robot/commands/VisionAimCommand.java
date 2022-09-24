package frc.robot.commands;

import java.io.Console;

import javax.xml.transform.TransformerConfigurationException;

import org.opencv.features2d.Feature2D;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShroudSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class VisionAimCommand extends CommandBase {
    private LimeLightSubsystem LL;
    private TurretSubsystem m_turret;
    private ShroudSubsystem m_shroud;
    private FeederSubsystem m_feeder;
    private HopperSubsystem m_hopper;

    private ShooterSubsystem m_shooter;

    public double LLHeight = 35;
    public double TargetHeight = 104;// inches
    public double LLAngle = 25;

    double limelightMountAngleDegrees = 25.0;
    double limelightLensHeightInches = 35;
    double goalHeightInches = 104.0;
    PIDController PIDVTurret = new PIDController(0.5, 0.6, 0);//.035
    PIDController PIDVShroud = new PIDController(0.005 , 0.014, 0.0000);// 0.0015, 0.012

    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.7591 , 0.13571, 0.035856);//new
    // private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.60213, 0.12494, 0.043843);
    private BangBangController shooterBang = new BangBangController(5);
    // private PIDController shooterPID = new PIDController(1.6, 0.015, 0);//1.511

    private OI m_oi;

    public VisionAimCommand(LimeLightSubsystem LL, TurretSubsystem m_turret, ShroudSubsystem m_shroud,
            ShooterSubsystem m_shooter, OI m_oi, FeederSubsystem m_feeder, HopperSubsystem m_hopper) {
        this.LL = LL;
        this.m_turret = m_turret;
        this.m_shroud = m_shroud;
        this.m_shooter = m_shooter;
        this.m_oi = m_oi;
        this.m_feeder = m_feeder;
        this.m_hopper = m_hopper;
        PIDVTurret.setTolerance(0);
        PIDVShroud.setTolerance(15);
        // shooterPID.setTolerance(20, 20);
        addRequirements(LL, m_turret, m_shroud, m_shooter);
    }

    @Override
    public void initialize() {
        m_turret.setTurret(0);
        m_shroud.setShroud(0);
        m_shooter.setShooter(0);
        m_feeder.setFeeder(0);
        m_hopper.setHopper(0);
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
        desiredPosition= -0.0016*Math.pow(distanceFromLimelightToGoalInches,2) + 1.4015 * distanceFromLimelightToGoalInches - 56.561;
        
    
        
        

        SmartDashboard.putNumber("DesiredShroud Position", desiredPosition);
        SmartDashboard.putNumber("Desired Shooter Speed", desiredSpeed);
        // if (m_oi.getAxis(1, Constants.Axes.LEFT_TRIGGER) > 0.5&&m_oi.getAxis(1, Constants.Axes.RIGHT_TRIGGER)>0.5) {
        //     //if top roller breaks
        //     if (LL.LLTable.getEntry("tx").getDouble(0) != -1) {
        //         m_turret.setTurret(-PIDVTurret.calculate(LL.LLTable.getEntry("tx").getDouble(0), 0));
        //         SmartDashboard.putNumber("VAlue", -PIDVTurret.calculate(LL.LLTable.getEntry("tx").getDouble(0), 0));
        //         desiredSpeed = 5500;
        //         desiredPosition = 0;
        //         m_shroud.setShroud(PIDVShroud.calculate(m_shroud.getShroudPosition(), desiredPosition));
        //         m_shooter.setShooter(shooterBang.calculate(m_shooter.getRate(), desiredSpeed)  
        //                + feedforward.calculate(desiredSpeed)*0.00043);
        //     }else{
        //         m_turret.setTurret(0);
        //         SmartDashboard.putNumber("VAlue", -PIDVTurret.calculate(LL.LLTable.getEntry("tx").getDouble(0), 0));
        //         desiredSpeed = 5500;
        //         desiredPosition = 0;
        //         m_shroud.setShroud(PIDVShroud.calculate(m_shroud.getShroudPosition(), desiredPosition));
        //         m_shooter.setShooter(shooterBang.calculate(m_shooter.getRate(), desiredSpeed)  
        //                + feedforward.calculate(desiredSpeed)*0.00043);
        //     }
        //     if (m_shooter.getRate() > desiredSpeed&&m_shroud.getShroudPosition()>desiredPosition-12&&m_shroud.getShroudPosition()<desiredPosition+12) {
        //         Constants.pushballs = true;
        //     } else {
        //         Constants.pushballs = false;
        //     }
        // }
        // else 
        SmartDashboard.putNumber("limelightx", LL.LLTable.getEntry("tx").getDouble(0));
        if(m_oi.getAxis(1, Constants.Axes.LEFT_TRIGGER)>0.5){
            if (LL.LLTable.getEntry("tx").getDouble(0) != -1) {
                SmartDashboard.putNumber("TurretOffsetVAlue", -PIDVTurret.calculate(HorizontalOffset, 0));
                // if(Math.abs(LL.LLTable.getEntry("tx").getDouble(0))<1){
                //     Constants.turning = false;
                // }
               m_turret.setTurret(0);
                // desiredPosition=160;
                // desiredSpeed=42;
                m_shroud.setShroud(PIDVShroud.calculate(m_shroud.getShroudPosition(), Math.max(0,desiredPosition)));
                // desiredSpeed = (desiredSpeed/2048)*10;
                double sped;
                if(distanceFromLimelightToGoalInches>290){
                    desiredSpeed= Math.min(38,0.0001*Math.pow(distanceFromLimelightToGoalInches, 2) - 0.0094*distanceFromLimelightToGoalInches+ 34);
                    sped = shooterBang.calculate(m_shooter.getRate(), desiredSpeed)+ (feedforward.calculate(desiredSpeed)/12);
                }else if(distanceFromLimelightToGoalInches<162){
                    desiredSpeed= Math.min(38,0.0001*Math.pow(distanceFromLimelightToGoalInches, 2) - 0.0094*distanceFromLimelightToGoalInches+ 22);
                    sped = shooterBang.calculate(m_shooter.getRate(), desiredSpeed+5)+ (feedforward.calculate(desiredSpeed+5)/12);
                }else{
                    
                desiredSpeed= Math.min(38,0.0001*Math.pow(distanceFromLimelightToGoalInches, 2) - 0.0094*distanceFromLimelightToGoalInches+ 30);
                sped = shooterBang.calculate(m_shooter.getRate(), desiredSpeed)+ (feedforward.calculate(desiredSpeed)/12);
            }
                // if((m_shooter.getRate()/2048)*10 < desiredSpeed){
                //     sp
                // }
                m_shooter.setShooter(sped);//*0.00047
                // m_shooter.setShooterVolt(Math.min(12,shooterPID.calculate((m_shooter.getRate()/2048)*10,desiredSpeed)+(feedforward.calculate(desiredSpeed))));
                
        SmartDashboard.putNumber("Current Shooter Speed",(m_shooter.getRate()/2048)*10);
                // m_shooter.setShooter(shooterBang.calculate(m_shooter.getRate(), desiredSpeed)  
                //        + feedforward.calculate(desiredSpeed)*0.00043);
            }else{
                m_turret.setTurret(0);
                m_shroud.setShroud(PIDVShroud.calculate(m_shroud.getShroudPosition(), desiredPosition));
                // m_shooter.setShooter(shooterBang.calculate(m_shooter.getRate(), desiredSpeed)  
                //        + feedforward.calculate(desiredSpeed)*0.00043);
                //desiredSpeed = 10;
                // m_shooter.setShooter(shooterPID.calculate(m_shooter.getRate(), desiredSpeed)/12. + (feedforward.calculate(desiredSpeed)*0.00043));
                m_shooter.setShooter(shooterBang.calculate(m_shooter.getRate(), desiredSpeed)+ (feedforward.calculate(desiredSpeed)*0.9/12));//*0.00047
                // m_shooter.setShooterVolt(shooterPID.calculate((m_shooter.getRate()/2048)*10,desiredSpeed)+(feedforward.calculate(desiredSpeed)*0.82));
            }
            //&&m_shroud.getShroudPosition()>desiredPosition-13&&m_shroud.getShroudPosition()<desiredPosition+13 

            if ((m_shooter.getRate()/2048)*10 > desiredSpeed) {
                Constants.pushballs = true;
                Constants.turning = true;
                m_feeder.setFeeder(.55);
        m_hopper.setHopper(.5);
            } else {
                Constants.pushballs = false;
                m_feeder.setFeeder(0);
        m_hopper.setHopper(0);
            }
        }
        else if (m_oi.getAxis(1, Constants.Axes.RIGHT_TRIGGER) > 0.5) {
            m_shooter.stopClimbing();
            //normal shooter
            if (LL.LLTable.getEntry("tx").getDouble(0) != -1) {
                SmartDashboard.putNumber("HorizontalOffesetVAlue", -PIDVTurret.calculate(HorizontalOffset, 0));
                // if(Math.abs(LL.LLTable.getEntry("tx").getDouble(0))<1){
                //     Constants.turning = false;
                // }
                // if(Constants.turning)
                SmartDashboard.putNumber("PID turret value", -PIDVTurret.calculate(LL.LLTable.getEntry("tx").getDouble(0), -3));
                m_turret.setTurretVolt(-PIDVTurret.calculate(LL.LLTable.getEntry("tx").getDouble(0), 1));
                // else
                // m_turret.setTurret(0);
                // desiredPosition=160;
                // desiredSpeed=42;
                // desiredPosition = 50;
                // desiredPosition = 55;
                // desiredPosition = 0;
                m_shroud.setShroud(PIDVShroud.calculate(m_shroud.getShroudPosition(), Math.max(0, desiredPosition)));

                 double sped;
                 //#region
                // if(distanceFromLimelightToGoalInches>430){
                //     desiredSpeed= Math.min(38,0.0001*Math.pow(distanceFromLimelightToGoalInches, 2) - 0.0094*distanceFromLimelightToGoalInches+ 31);
                //     sped = shooterBang.calculate(m_shooter.getRate(), desiredSpeed)+ (feedforward.calculate(desiredSpeed)*1./12);
                // }
                // else if(distanceFromLimelightToGoalInches>340){
                //     desiredSpeed= Math.min(38,0.0001*Math.pow(distanceFromLimelightToGoalInches, 2) - 0.0094*distanceFromLimelightToGoalInches+ 32);
                //     sped = shooterBang.calculate(m_shooter.getRate(), desiredSpeed)+ (feedforward.calculate(desiredSpeed)*1./12);
                // }
                // else if(distanceFromLimelightToGoalInches>290){
                //     desiredSpeed= Math.min(38,0.0001*Math.pow(distanceFromLimelightToGoalInches, 2) - 0.0094*distanceFromLimelightToGoalInches+ 31);
                //     sped = shooterBang.calculate(m_shooter.getRate(), desiredSpeed)+ (feedforward.calculate(desiredSpeed)*1./12);
                // }else if(distanceFromLimelightToGoalInches<162){
                //     desiredSpeed= Math.min(38,0.0001*Math.pow(distanceFromLimelightToGoalInches, 2) - 0.0094*distanceFromLimelightToGoalInches+ 23);
                //     sped = shooterBang.calculate(m_shooter.getRate(), desiredSpeed+5)+ (feedforward.calculate(desiredSpeed+5)*1.00/12);
                // }else{
                    //#endregion
                if(distanceFromLimelightToGoalInches<150){
                desiredSpeed = Math.min(40,-0.0001*Math.pow(distanceFromLimelightToGoalInches, 2) + 0.1517*distanceFromLimelightToGoalInches+ 11.7);
                }else if(distanceFromLimelightToGoalInches<280){
                    desiredSpeed = Math.min(40,-0.0001*Math.pow(distanceFromLimelightToGoalInches, 2) + 0.1517*distanceFromLimelightToGoalInches+ 8.5);
                }else{
                    desiredSpeed = Math.min(40,-0.0001*Math.pow(distanceFromLimelightToGoalInches, 2) + 0.1517*distanceFromLimelightToGoalInches+ 6);
                }
                // desiredSpeed = 39;
                sped = shooterBang.calculate(m_shooter.getRate(), desiredSpeed)+ (feedforward.calculate(desiredSpeed)*1.00/12);
                SmartDashboard.putNumber("calculated sped", sped);
                // }    
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
            
            
                // if((m_shooter.getRate()/2048)*10 < desiredSpeed){
                //     sp
                // }
                m_shooter.setShooter(sped);//*0.00047
                // m_shooter.setShooterVolt(Math.min(12,shooterPID.calculate((m_shooter.getRate()/2048)*10,desiredSpeed)+(feedforward.calculate(desiredSpeed))));
                
                SmartDashboard.putNumber("Current Shooter Speed",(m_shooter.getRate()/2048)*10);
                SmartDashboard.putNumber("Current Shroud",(m_shroud.getShroudPosition()));
                // m_shooter.setShooter(shooterBang.calculate(m_shooter.getRate(), desiredSpeed)  
                //        + feedforward.calculate(desiredSpeed)*0.00043);
            }else{
                // if(m_turret.getTurretRot()<5.5){
                //     m_turret.setTurret(0.2);
                // }else if(m_turret.getTurretRot()>6.5){
                //     m_turret.setTurret(-0.2);
                // }

               // m_shroud.setShroud(PIDVShroud.calculate(m_shroud.getShroudPosition(), desiredPosition));
                // m_shooter.setShooter(shooterBang.calculate(m_shooter.getRate(), desiredSpeed)  
                //        + feedforward.calculate(desiredSpeed)*0.00043);
                //desiredSpeed = 10;
                // m_shooter.setShooter(shooterPID.calculate(m_shooter.getRate(), desiredSpeed)/12. + (feedforward.calculate(desiredSpeed)*0.00043));
                // m_shooter.setShooter(shooterBang.calculate(m_shooter.getRate(), 17)+ (feedforward.calculate(desiredSpeed)*0.9/12));//*0.00047
                // m_shooter.setShooterVolt(shooterPID.calculate((m_shooter.getRate()/2048)*10,desiredSpeed)+(feedforward.calculate(desiredSpeed)*0.82));
            }
            // &&m_shroud.getShroudPosition()>desiredPosition-15&&m_shroud.getShroudPosition()<desiredPosition+15
            if ((m_shooter.getRate()/2048)*10 > desiredSpeed) {
                Constants.pushballs = true;
                Constants.turning = true;
                if(distanceFromLimelightToGoalInches>350){
                    m_feeder.setFeeder(0.3);
                }
                else if(distanceFromLimelightToGoalInches>290){
                    m_feeder.setFeeder(0.4);
                }else{
                m_feeder.setFeeder(.45);
                }
        m_hopper.setHopper(.8);
            } else {
                Constants.pushballs = false;
                m_feeder.setFeeder(0);
        m_hopper.setHopper(0);
            }
        }
        // else if (m_oi.getAxis(0, Constants.Axes.RIGHT_TRIGGER) > 0.5) {
            //lower hub shooter
        //     desiredPosition = 100;
        //     desiredSpeed = 4200;
        //     if (LL.LLTable.getEntry("tx").getDouble(0) != -1) {
        //         SmartDashboard.putNumber("VAlue", -PIDVTurret.calculate(LL.LLTable.getEntry("tx").getDouble(0), 0));
        //         m_turret.setTurret(-PIDVTurret.calculate(LL.LLTable.getEntry("tx").getDouble(0), 0));
        //         m_shroud.setShroud(PIDVShroud.calculate(m_shroud.getShroudPosition(), desiredPosition));
        //         m_shooter.setShooter(shooterBang.calculate(m_shooter.getRate(), desiredSpeed)  
        //                + feedforward.calculate(desiredSpeed)*0.00043);
        //     }
        //     else{
        //         m_turret.setTurret(0);
        //         m_shroud.setShroud(PIDVShroud.calculate(m_shroud.getShroudPosition(), desiredPosition));
        //         m_shooter.setShooter(shooterBang.calculate(m_shooter.getRate(), desiredSpeed)  
        //                + feedforward.calculate(desiredSpeed)*0.00043);
        //     }
        //     if (m_shooter.getRate() > desiredSpeed&&m_shroud.getShroudPosition()>desiredPosition-15&&m_shroud.getShroudPosition()<desiredPosition+15) {
        //         Constants.pushballs = true;
        //     } else {
        //         Constants.pushballs = false;
        //     }
        // }
      
        else if(m_oi.getButton(1, Constants.Buttons.LEFT_BUMPER).get()){
            m_shroud.setShroud(PIDVShroud.calculate(m_shroud.getShroudPosition(), 0));
            desiredSpeed = 20;
            m_shooter.setShooter(shooterBang.calculate(m_shooter.getRate(), desiredSpeed)+ (feedforward.calculate(desiredSpeed)/12));
            m_feeder.setFeeder(0.5);
        }
        else {
        
                // m_shooter.stopShooter();
                // m_shooter.setShooterVolt(feedforward.calculate((m_shooter.getRate()/2048)*10,20)*0.8);
                desiredSpeed = 10;
                if(Constants.climbing){
                   
                }
                //shooterPID.calculate((m_shooter.getRate()/2048)*10,desiredSpeed)+(feedforward.calculate((m_shooter.getRate()/2048)*10,desiredSpeed)*0.81)
                m_shooter.setShooter(shooterBang.calculate(m_shooter.getRate(), desiredSpeed)+ (feedforward.calculate(desiredSpeed)/12));//*0.00047
                // m_shooter.setShooterVolt(shooterPID.calculate((m_shooter.getRate()/2048)*10,desiredSpeed)+(feedforward.calculate(desiredSpeed)*0.81));
                
                
                // m_shooter.setShooter(shooterBang.calculate(m_shooter.getRate(), 1750)  +0.0007*(feedforward.calculate(1750)) );
            m_turret.stopTurret();
            m_shroud.stopShroud();
            m_feeder.setFeeder(0);
        m_hopper.setHopper(0);
    }
    // if(m_oi.getButton(0, Constants.Buttons.A_BUTTON).getAsBoolean()){

    // }
    if(m_oi.getButton(1, Constants.Buttons.RIGHT_BUMPER).get()){
        m_turret.setTurretVolt(PIDVTurret.calculate(m_turret.getTurretRot()*2, 0));
    }else if(m_oi.getPovButton(1, 90).get()){
        m_turret.setTurret(0.2);
    }else if(m_oi.getPovButton(1, 270).get()){
        m_turret.setTurret(-0.2);
    }

        SmartDashboard.putNumber("desired speed"    , desiredSpeed);
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
        return false;
    }

}
