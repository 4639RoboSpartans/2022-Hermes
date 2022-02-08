package frc.robot.commands.AutonomousRoutines;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class Path3 extends CommandBase {
    public DriveSubsystem m_drive;
    public IntakeSubsystem m_intake;
    public HopperSubsystem m_hopper;
    public FeederSubsystem m_feeder;
    public TurretSubsystem m_turret;
    public LimeLightSubsystem m_LL;
    public ShooterSubsystem m_shooter;
    public Path3(DriveSubsystem m_drive, IntakeSubsystem m_intake, HopperSubsystem m_hopper, FeederSubsystem m_feeder, TurretSubsystem m_turret, LimeLightSubsystem m_LL, ShooterSubsystem m_shooter){
        this.m_drive = m_drive;
        this.m_intake = m_intake;
        this.m_hopper = m_hopper;
        this.m_feeder = m_feeder;
        this.m_turret = m_turret;
        this.m_LL = m_LL;
        this.m_shooter = m_shooter;
        addRequirements(m_drive, m_intake, m_hopper, m_feeder, m_turret,m_LL, m_shooter);
    }
}
