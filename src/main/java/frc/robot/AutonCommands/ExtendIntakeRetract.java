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
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ExtendIntakeRetract extends CommandBase {
     public IntakeSubsystem m_intake;
     public HopperSubsystem m_hopper;
     Timer time;
    public ExtendIntakeRetract(IntakeSubsystem m_intake, HopperSubsystem m_hopper) {
        this.m_hopper=m_hopper;
        this.m_intake=m_intake;
        time = new Timer();
        addRequirements(m_intake,m_hopper);
    }

    @Override
    public void initialize() {
        time.start();
        m_intake.retractPistons();
        
    }
    @Override 
    public void execute(){
        m_intake.setIntake(0.7);
        m_hopper.setHopper(0.5);
        
    }
    @Override
    public void end(boolean fin){
        m_intake.setIntake(0);
        m_hopper.setHopper(0);
        m_intake.extendPistons();
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
