package frc.robot.commands;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShroudSubsystem;

public class LowerHubShooter extends CommandBase{
    private ShooterSubsystem m_shooter ;
    private FeederSubsystem m_feeder ;
    private HopperSubsystem m_hopper;
    private ShroudSubsystem m_shroud;
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.64665, 0.10772, 0.037027);
    private BangBangController shooterBang = new BangBangController(5);
    PIDController PIDVShroud = new PIDController(0.0017, 0.00049, 0.0000);// 0.0014, 0.0044,0.00
    // private PIDController shooterPID = new PIDController(0.00026,0.00006,0.0);
    private double desiredSpeed =4000;
    public LowerHubShooter(ShooterSubsystem m_shooter,FeederSubsystem m_feeder, HopperSubsystem m_hopper, ShroudSubsystem m_shroud){
        this.m_shooter = m_shooter;
        this.m_feeder = m_feeder;
        this.m_hopper = m_hopper;
        this.m_shroud = m_shroud;
        // shooterPID.setTolerance(50);
        addRequirements(m_shooter, m_feeder, m_hopper, m_shroud);
    }
    
    public double calculate(){
        //return shooterPID.calculate(m_shooter.getRate(), desiredSpeed);
        
        return shooterBang.calculate(m_shooter.getRate(),desiredSpeed)+0.0006*feedforward.calculate(desiredSpeed);
    }
    @Override
    public void initialize(){
        m_shooter.setShooter(0);
        m_feeder.setFeeder(0);
        m_hopper.setHopper(0);
        PIDVShroud.setTolerance(7);
    }
    @Override
    public void execute(){
        //m_shooter.setShooter(0.5);
        m_shooter.setShooter(calculate());
                if(m_shooter.getRate()>=desiredSpeed*1.1){
                    m_feeder.setFeeder(0.60);
                    m_hopper.setHopper(0.5);
                    m_shroud.setShroud(PIDVShroud.calculate(m_shroud.getShroudPosition(), 200));
        }else{
            m_feeder.stop();
            m_hopper.stopHopper();
            m_shroud.setShroud(0);
        }
    }
    @Override
    public void end(boolean interrupted){
        m_shooter.stopShooter();
        m_feeder.stop();
        m_hopper.stopHopper();
        m_shroud.stopShroud();
    }
    @Override 
    public boolean isFinished(){
        return false;
    }

}
