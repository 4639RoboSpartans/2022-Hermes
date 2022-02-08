package frc.robot.commands.AutonomousRoutines;

import java.sql.Driver;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class Path1 extends CommandBase {
    public DriveSubsystem m_drive;
    public IntakeSubsystem m_intake;
    public HopperSubsystem m_hopper;
    public FeederSubsystem m_feeder;
    public TurretSubsystem m_turret;
    public LimeLightSubsystem m_LL;
    public ShooterSubsystem m_shooter;
    public Path1(DriveSubsystem m_drive, IntakeSubsystem m_intake, HopperSubsystem m_hopper, FeederSubsystem m_feeder, TurretSubsystem m_turret, LimeLightSubsystem m_LL, ShooterSubsystem m_shooter){
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
