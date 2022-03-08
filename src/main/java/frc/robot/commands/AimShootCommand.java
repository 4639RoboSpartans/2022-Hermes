package frc.robot.commands;

import java.sql.Time;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShroudSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class AimShootCommand extends CommandBase{
    private ShooterSubsystem m_shooter;
    private FeederSubsystem m_feeder;
    private HopperSubsystem m_hopper;
    private TurretSubsystem m_turret;
    private ShroudSubsystem m_shroud;
    private LimeLightSubsystem m_ll;
    Timer time;
    public double LLHeight = 23.5;
    public double TargetHeight=104;//inches
    public double LLAngle = 33;
    // private DriveSubsystem m_drive;
    //add feedforward mechanism!!!!!!!!
    PIDController PIDVTurret = new PIDController(0.032, 0, 0);
    PIDController PIDVShroud = new PIDController(0.0014, 0.0004, 0.0000);//0.0014, 0.0044,0.00
    public AimShootCommand(ShooterSubsystem m_shooter, FeederSubsystem m_feeder, HopperSubsystem m_hopper, LimeLightSubsystem m_LL, TurretSubsystem m_turret, ShroudSubsystem m_shroud){
        this.m_shooter = m_shooter;
        this.m_feeder = m_feeder;
        this.m_hopper = m_hopper;
        this.m_turret = m_turret;
        this.m_shroud = m_shroud;
        this.m_ll = m_LL;
        time = new Timer();
        addRequirements(m_shooter, m_feeder, m_hopper, m_turret, m_shroud);
    }
    @Override
    public void initialize() {
        time.start();
        
    }
    @Override 
    public void execute(){
        NetworkTableEntry ty = m_ll.LLTable.getEntry("ty");
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
    m_shroud.setShroud(PIDVShroud.calculate(m_shroud.getShroudPosition(), desiredPosition));
        m_turret.setTurret(-PIDVTurret.calculate(m_ll.LLTable.getEntry("tx").getDouble(0), 0));
    }
    @Override
    public void end(boolean fin){
        m_hopper.setHopper(0);
        //m_intake.extendPistons();
    }
    @Override
    public boolean isFinished(){
        if(time.get()>4){
            time.stop();
            return true;
        }
        return false;

    }
}
