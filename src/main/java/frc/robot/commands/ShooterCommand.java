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

public class ShooterCommand extends CommandBase{
    private ShooterSubsystem m_shooter ;
    private FeederSubsystem m_feeder ;
    private HopperSubsystem m_hopper;
    private LimeLightSubsystem LL ;
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.64665, 0.10772, 0.037027);
    private BangBangController shooterBang = new BangBangController(5);
double limelightMountAngleDegrees = 33.0;
double limelightLensHeightInches = 23.5;
double goalHeightInches = 104.0;
    // private PIDController shooterPID = new PIDController(0.00026,0.00006,0.0);
    private double desiredSpeed =6000;
    public ShooterCommand(ShooterSubsystem m_shooter, LimeLightSubsystem LL, FeederSubsystem m_feeder, HopperSubsystem m_hopper){
        this.m_shooter = m_shooter;
        this.m_feeder = m_feeder;
        this.m_hopper = m_hopper;
        this.LL = LL;
        // shooterPID.setTolerance(50);
        addRequirements(m_shooter, LL);
    }
    public double calculate(){
        //return shooterPID.calculate(m_shooter.getRate(), desiredSpeed);
        
double targetOffsetAngle_Vertical = LL.LLTable.getEntry("ty").getDouble(0.0);


double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

//calculate distance
double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians);
desiredSpeed = -0.0045*Math.pow(distanceFromLimelightToGoalInches,2) + 8.6897*distanceFromLimelightToGoalInches + 4950;
        
return shooterBang.calculate(m_shooter.getRate(),desiredSpeed)*2+0.0006*feedforward.calculate(desiredSpeed);
    }
    @Override
    public void initialize(){
        m_shooter.setShooter(0);
        m_feeder.setFeeder(0);
        m_hopper.setHopper(0);
    }
    @Override
    public void execute(){
        //m_shooter.setShooter(0.5);
        m_shooter.setShooter(calculate());
                if(m_shooter.getRate()>=desiredSpeed*1.1){
                    m_feeder.setFeeder(0.60);
                    m_hopper.setHopper(0.5);

            
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
