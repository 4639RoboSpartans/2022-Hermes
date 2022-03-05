package frc.robot.commands;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends CommandBase{
    private ShooterSubsystem m_shooter ;
    private FeederSubsystem m_feeder ;
    private HopperSubsystem m_hopper;
    private LimeLightSubsystem LL ;
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.00575, 0.0010832, 0.003574);
    private BangBangController shooterBang = new BangBangController(5);
    private PIDController shooterPID = new PIDController(0.00041,0.00009,0.0);
    private double desiredSpeed =7000;
    public ShooterCommand(ShooterSubsystem m_shooter, FeederSubsystem m_feeder, LimeLightSubsystem LL, HopperSubsystem m_hopper){
        this.m_shooter = m_shooter;
        this.m_feeder = m_feeder;
        this.m_hopper = m_hopper;
        this.LL = LL;
        shooterPID.setTolerance(10);
        addRequirements(m_shooter,m_feeder, LL);
    }
    public double calculate(){
        return shooterPID.calculate(m_shooter.getRate(), desiredSpeed);
        // return shooterBang.calculate(m_shooter.getRate(),2000)+feedforward.calculate(2000);
    }
    @Override
    public void initialize(){
        m_shooter.setShooter(0);
        m_feeder.setFeeder(0);
        m_hopper.setHopper(0);
    }
    @Override
    public void execute(){
        m_shooter.setShooter(calculate());
        if(m_shooter.getRate()>=desiredSpeed){
            m_feeder.setFeeder(0.4);
            m_hopper.setHopper(0.4);
        }else{
            m_feeder.stop();
            m_hopper.stopHopper();
        }
    }
    @Override
    public void end(boolean interrupted){
        m_shooter.stopShooter();
        m_feeder.stop();
        m_hopper.stopHopper();
    }
    @Override 
    public boolean isFinished(){
        return false;
    }

}
