package frc.robot.commands;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends CommandBase{
    private ShooterSubsystem m_shooter ;
    private FeederSubsystem m_feeder ;
    private LimeLightSubsystem LL ;
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0, 0);
    private BangBangController shooterBang = new BangBangController(5);
    public ShooterCommand(ShooterSubsystem m_shooter, FeederSubsystem m_feeder, LimeLightSubsystem LL){
        this.m_shooter = m_shooter;
        this.m_feeder = m_feeder;
        this.LL = LL;
        addRequirements(m_shooter,m_feeder, LL);
    }
    public double calculate(){
        return shooterBang.calculate(m_shooter.getRate(),0)+0.9*feedforward.calculate(0/*formula*/);
    }
    @Override
    public void initialize(){
        m_shooter.setShooter(0);
        //m_feeder.setFeeder(0);
    }
    @Override
    public void execute(){
        m_shooter.setShooter(/*calculate()*/0.6);
        if(m_shooter.getRate()>=3700){
            m_feeder.setFeeder(0.4);
        }
        // if(calculate()>=0/*formla*/-100&&calculate()<=0/*formula*/+100){
        //     m_feeder.setFeeder(0.2);
        // }else{
        //     m_feeder.stop();
        // }
    }
    @Override
    public void end(boolean interrupted){
        m_shooter.stopShooter();
        //m_feeder.stop();
    }
    @Override 
    public boolean isFinished(){
        return false;
    }

}
