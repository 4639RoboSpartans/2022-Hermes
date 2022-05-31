package frc.robot.AutonCommands;

import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class NIntake extends CommandBase {
     public IntakeSubsystem m_intake;
     public HopperSubsystem m_hopper;
     Timer time;
    double tim;
    double timb;
    

    public NIntake(IntakeSubsystem m_intake, HopperSubsystem m_hopper, double tim, double timb) {
        this.m_hopper=m_hopper;
        this.m_intake=m_intake;
        time = new Timer();
        this.tim = tim;
        this.timb = timb;
        addRequirements(m_intake,m_hopper);
    }

    @Override
    public void initialize() {
        time.start();
        m_intake.extendPistons();
        
    }
    @Override 
    public void execute(){
        if(time.get()>timb){
        m_intake.setIntake(0.5);
        m_hopper.setHopper(0.5);
        }
        
    }
    @Override
    public void end(boolean fin){
        m_intake.setIntake(0);
        m_hopper.setHopper(0);
        m_intake.retractPistons();
    }
    @Override
    public boolean isFinished(){
        if(time.get()>tim){
            time.stop();
            return true;
        }
        return false;

    }
}
