// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ClimberBackwardCommand;
import frc.robot.commands.ClimberForwardCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.OutTakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.VisionAimCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShroudSubsystem;
import frc.robot.subsystems.TurretSubsystem;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class RobotContainer {
  public DriveSubsystem m_drive = new DriveSubsystem();
  public IntakeSubsystem m_intake = new IntakeSubsystem();
  // public ClimberSubsystem m_climber = new ClimberSubsystem();
  public FeederSubsystem m_feeder = new FeederSubsystem();
  public HopperSubsystem m_hopper = new HopperSubsystem();
  // public LimeLightSubsystem m_LL = new LimeLightSubsystem();
  // public ShooterSubsystem m_shooter = new ShooterSubsystem();
  // public ShroudSubsystem m_shroud = new ShroudSubsystem();
  // public TurretSubsystem m_turret = new TurretSubsystem();

  public SendableChooser<Command> m_chooser = new SendableChooser<>();
  public OI m_oi = new OI();
  private String path31;
  private String path32;
  Trajectory traj = new Trajectory();
  Trajectory traj2 = new Trajectory();
  public RobotContainer() {
    path31 = "paths/path3part1.wpilib.json";
    path32 = "paths/path3part2.wpilib.json";
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path31);
      Path trajectoryPath2 = Filesystem.getDeployDirectory().toPath().resolve(path32);
      traj = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      traj2 = TrajectoryUtil.fromPathweaverJson(trajectoryPath2);
   } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + path31, ex.getStackTrace());
   }
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    m_drive.setDefaultCommand(new DriveCommand(m_drive, m_oi));
    // m_oi.getPovButton(1, 0).whileHeld(new RunCommand(() ->
    // m_intake.extendPistons(), m_intake));
    // m_oi.getPovButton(1, 180).whileHeld(new RunCommand(() ->
    // m_intake.retractPistons(), m_intake));
    m_oi.getButton(1, Constants.Buttons.X_BUTTON).whileHeld(new
    IntakeCommand(m_intake, m_hopper, m_feeder));
    // m_oi.getButton(1, Constants.Buttons.A_BUTTON).whileHeld(new
    // OutTakeCommand(m_intake, m_hopper, m_feeder));
    // m_oi.getButton(1, Constants.Buttons.LEFT_BUMPER).whileHeld(new
    // VisionAimCommand(m_LL, m_turret, m_shroud, m_drive));
    // m_oi.getButton(1, Constants.Buttons.RIGHT_BUMPER).whileHeld(new
    // ShooterCommand(m_shooter, m_feeder, m_LL));
    // m_oi.getButton(1, Constants.Buttons.Y_BUTTON).whileHeld(new
    // ClimberForwardCommand(m_climber));
    // m_oi.getButton(1, Constants.Buttons.B_BUTTON).whileHeld(new
    // ClimberBackwardCommand(m_climber));
  }

  public Command getAutonomousCommand() {
    RamseteCommand ramseteCommand = new RamseteCommand(
        traj,
        m_drive::getPose,
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
        new SimpleMotorFeedforward(
            Constants.ksVolts,
            Constants.kvVoltSecondsPerMeter,
            Constants.kaVoltSecondsSquaredPerMeter),
        Constants.kDriveKinematics,
        m_drive::getWheelSpeeds,
        new PIDController(Constants.kPDriveVel, 0, 0),
        new PIDController(Constants.kPDriveVel, 0, 0),
        m_drive::tankDriveVolts,
        m_drive);
    m_drive.resetOdometry(traj.getInitialPose());

    RamseteCommand ramseteCommand2 = new RamseteCommand(
        traj2,
        m_drive::getPose,
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
        new SimpleMotorFeedforward(
            Constants.ksVolts,
            Constants.kvVoltSecondsPerMeter,
            Constants.kaVoltSecondsSquaredPerMeter),
        Constants.kDriveKinematics,
        m_drive::getWheelSpeeds,
        new PIDController(Constants.kPDriveVel, 0, 0),
        new PIDController(Constants.kPDriveVel, 0, 0),
        m_drive::tankDriveVolts,
        m_drive);
    m_drive.resetOdometry(traj.getInitialPose());

    
    
    return ramseteCommand.andThen(() -> m_drive.tankDriveVolts(0, 0))
    .andThen(ramseteCommand2.andThen(() -> m_drive.tankDriveVolts(0, 0)));
  }
}

// var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
// new SimpleMotorFeedforward(
// Constants.ksVolts,
// Constants.kvVoltSecondsPerMeter,
// Constants.kaVoltSecondsSquaredPerMeter),
// Constants.kDriveKinematics,
// 10);
// // TrajectoryConfig config = new TrajectoryConfig(
// Constants.kMaxSpeedMetersPerSecond,
// Constants.kMaxAccelerationMetersPerSecondSquared)
// .setKinematics(Constants.kDriveKinematics)
// .addConstraint(autoVoltageConstraint);
// Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
// // Start at the origin facing the +X direction
// new Pose2d(0, 0, new Rotation2d(0)),
// // Pass through these two interior waypoints, making an 's' curve path
// List.of(new Translation2d(4,0), new Translation2d(4,4), new
// Translation2d(0,4)),
// // End 3 meters straight ahead of where we started, facing forward
// new Pose2d(0,0 , new Rotation2d(0)),
// // Pass config
// config);
