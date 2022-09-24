// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.AutonCommands.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import javax.lang.model.element.ExecutableElement;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

public class RobotContainer {
  public OI m_oi = new OI();
  public DriveSubsystem m_drive = new DriveSubsystem();
  public IntakeSubsystem m_intake = new IntakeSubsystem();
  public ClimberSubsystem m_climber = new ClimberSubsystem();
  public FeederSubsystem m_feeder = new FeederSubsystem();
  public HopperSubsystem m_hopper = new HopperSubsystem();
  public LimeLightSubsystem m_LL = new LimeLightSubsystem();
  public ShooterSubsystem m_shooter = new ShooterSubsystem();
  public ShroudSubsystem m_shroud = new ShroudSubsystem();
  public TurretSubsystem m_turret = new TurretSubsystem();

  public DriveCommand drive = new DriveCommand(m_drive, m_oi);
  public IntakeCommand intake = new IntakeCommand(m_intake, m_hopper, m_feeder, m_oi);
  public OutTakeCommand outtake = new OutTakeCommand(m_intake, m_hopper, m_feeder);
  public ShooterCommand shooter = new ShooterCommand(m_shooter, m_LL, m_feeder, m_hopper);
  public VisionAimCommand LL = new VisionAimCommand(m_LL, m_turret, m_shroud, m_shooter, m_oi, m_feeder, m_hopper);
  public ClimberForwardCommand climberforward = new ClimberForwardCommand(m_climber);
  public ClimberBackwardCommand climberbackward = new ClimberBackwardCommand(m_climber);
  public ClimberRightForward ClimberRF = new ClimberRightForward(m_climber);
  public ClimberLeftForward ClimberLF = new ClimberLeftForward(m_climber);
  public ClimberRightBackward ClimberRB = new ClimberRightBackward(m_climber);
  public ClimberLeftBackward ClimberLB = new ClimberLeftBackward(m_climber);
  public LowerHubShooter LowerHubShooter = new LowerHubShooter(m_shooter, m_feeder, m_hopper, m_shroud);
  public TurretCommand turret = new TurretCommand(m_turret, m_oi);
  public TurretCommandR turretr = new TurretCommandR(m_turret);

  public ExtendIntake EIntake = new ExtendIntake(m_intake, m_hopper);
  public ExtendIntake2 EIntake2 = new ExtendIntake2(m_intake, m_hopper);
  public NIntake nintake = new NIntake(m_intake, m_hopper, 2, 0);
  public NIntake n2intake = new NIntake(m_intake, m_hopper, 3.5, 0);
  public NIntake n3intake = new NIntake(m_intake, m_hopper, 3.5, 0);
  public IntakeRetract IntakeR = new IntakeRetract(m_intake, m_hopper);
  public ExtendIntakeRetract EIntakeR = new ExtendIntakeRetract(m_intake, m_hopper);
  public ShooterAuto autonShooter = new ShooterAuto(m_shooter, m_LL, m_feeder, m_hopper);
  public VisionAuto autonVision = new VisionAuto(m_LL, m_turret, m_shroud, m_shooter, m_oi, m_feeder, m_hopper,
      m_intake, false, 2);
  public ShooterAuto2 autonShooter2 = new ShooterAuto2(m_shooter, m_LL, m_feeder, m_hopper);
  public VisionAuto autonVision2 = new VisionAuto(m_LL, m_turret, m_shroud, m_shooter, m_oi, m_feeder, m_hopper,
      m_intake, true, 4);
  public SendableChooser<String> m_chooser = new SendableChooser<>();

  private String path11;
  private String path12;
  private String path13;
  private String path21;
  private String path22;
  private String path23;
  private String path24;
  private String path31;
  private String path32;
  private String path33;
  private String path34;

  Trajectory traj11 = new Trajectory();
  Trajectory traj12 = new Trajectory();
  Trajectory traj13 = new Trajectory();

  Trajectory traj21 = new Trajectory();
  Trajectory traj22 = new Trajectory();
  Trajectory traj23 = new Trajectory();
  Trajectory traj24 = new Trajectory();

  Trajectory traj31 = new Trajectory();
  Trajectory traj32 = new Trajectory();
  Trajectory traj33 = new Trajectory();
  Trajectory traj34 = new Trajectory();

  AutonPath aPath11, aPath12, aPath13;
  AutonPath aPath21, aPath22, aPath23;
  AutonPath aPath31, aPath32, aPath33;
  AutonPath aPath41, aPath42;

  Command path1, path2, path3, test;

  public RobotContainer() {
    m_chooser.setDefaultOption("Climber5", "3L");
    m_chooser.addOption("Climber4", "3M");
    m_chooser.addOption("Climber2", "3S");
    m_chooser.addOption("Middle4", "2M");
    m_chooser.addOption("Middle2", "2S");
    SmartDashboard.putData(m_chooser);

    path11 = "paths/path1part1.wpilib.json";
    path12 = "paths/path1part2.wpilib.json";
    path13 = "paths/path1part3.wpilib.json";

    path21 = "paths/path2part1.wpilib.json";
    path22 = "paths/path2part2.wpilib.json";
    path23 = "paths/path2part3.wpilib.json";
    path24 = "paths/path2part4.wpilib.json";

    path31 = "paths/path3part1.wpilib.json";
    path32 = "paths/path3part2.wpilib.json";
    path33 = "paths/path3part3.wpilib.json";
    path34 = "paths/path3part4.wpilib.json";
    try {
      Path trajectoryPath11 = Filesystem.getDeployDirectory().toPath().resolve(path11);
      traj11 = TrajectoryUtil.fromPathweaverJson(trajectoryPath11);
      Path trajectoryPath12 = Filesystem.getDeployDirectory().toPath().resolve(path12);
      traj12 = TrajectoryUtil.fromPathweaverJson(trajectoryPath12);
      Path trajectoryPath13 = Filesystem.getDeployDirectory().toPath().resolve(path13);
      traj13 = TrajectoryUtil.fromPathweaverJson(trajectoryPath13);

      Path trajectoryPath21 = Filesystem.getDeployDirectory().toPath().resolve(path21);
      traj21 = TrajectoryUtil.fromPathweaverJson(trajectoryPath21);
      Path trajectoryPath22 = Filesystem.getDeployDirectory().toPath().resolve(path22);
      traj22 = TrajectoryUtil.fromPathweaverJson(trajectoryPath22);
      Path trajectoryPath23 = Filesystem.getDeployDirectory().toPath().resolve(path23);
      traj23 = TrajectoryUtil.fromPathweaverJson(trajectoryPath23);
      Path trajectoryPath24 = Filesystem.getDeployDirectory().toPath().resolve(path24);
      traj24 = TrajectoryUtil.fromPathweaverJson(trajectoryPath24);

      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path31);
      traj31 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      Path trajectoryPath2 = Filesystem.getDeployDirectory().toPath().resolve(path32);
      traj32 = TrajectoryUtil.fromPathweaverJson(trajectoryPath2);
      Path trajectoryPath3 = Filesystem.getDeployDirectory().toPath().resolve(path33);
      traj33 = TrajectoryUtil.fromPathweaverJson(trajectoryPath3);
      Path trajectoryPath4 = Filesystem.getDeployDirectory().toPath().resolve(path34);
      traj34 = TrajectoryUtil.fromPathweaverJson(trajectoryPath4);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + path31, ex.getStackTrace());
    }
    m_drive.resetOdometry(traj31.getInitialPose());
    aPath11 = new AutonPath("pathplanner/generatedJSON/Path1Part1.wpilib.json");
    aPath12 = new AutonPath("pathplanner/generatedJSON/Path1Part2.wpilib.json");
    aPath13 = new AutonPath("pathplanner/generatedJSON/Path1Part3.wpilib.json");

    aPath21 = new AutonPath("pathplanner/generatedJSON/Path2Part1.wpilib.json");
    aPath22 = new AutonPath("pathplanner/generatedJSON/Path2Part2.wpilib.json");
    aPath23 = new AutonPath("pathplanner/generatedJSON/Path2Part3.wpilib.json");

    aPath31 = new AutonPath("pathplanner/generatedJSON/Path3Part1.wpilib.json");
    aPath32 = new AutonPath("pathplanner/generatedJSON/Path3Part2.wpilib.json");
    aPath33 = new AutonPath("pathplanner/generatedJSON/Path3Part3.wpilib.json");

    aPath41 = new AutonPath("pathplanner/generatedJSON/Path4Part1.wpilib.json");
    aPath42 = new AutonPath("pathplanner/generatedJSON/Path4Part2.wpilib.json");

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    
    m_drive.setDefaultCommand(drive);
    m_LL.setDefaultCommand(LL);
    m_oi.getPovButton(1, 180).whenPressed(new RunCommand(() -> m_intake.extendPistons(), m_intake));
    m_oi.getPovButton(1, 0).whenPressed(new RunCommand(() -> m_intake.retractPistons(), m_intake));
    m_oi.getButton(1, Constants.Buttons.X_BUTTON).whileHeld(intake);
    m_oi.getButton(1, Constants.Buttons.A_BUTTON).whileHeld(outtake);
    // m_oi.getButton(1,Constants.Buttons.LEFT_BUMPER).whileHeld(turret);
    // m_oi.getButton(1,Constants.Buttons.RIGHT_BUMPER).whileHeld(turret1);
    m_oi.getButton(1, Constants.Buttons.RIGHT_BUMPER).whileHeld(LL);
    m_oi.getPovButton(0, 180).whenPressed(new RunCommand(() -> m_climber.extendPistons(), m_climber));
    m_oi.getPovButton(0, 0).whenPressed(new RunCommand(() -> m_climber.retractPistons(), m_climber));

    // m_oi.getButton(1,
    // Constants.Buttons.Y_BUTTON).whileHeld(()->m_climber.extendPistons());
    // m_oi.getButton(1,
    // Constants.Buttons.A_BUTTON).whileHeld(()->m_climber.retractPistons());

    m_oi.getButton(0, Constants.Buttons.LEFT_BUMPER).whileHeld(climberforward);
    m_oi.getButton(0, Constants.Buttons.LEFT_BUMPER)
        .whenPressed(new RunCommand(() -> m_shooter.startClimbing(), m_shooter));
    m_oi.getButton(0, Constants.Buttons.RIGHT_BUMPER).whileHeld(climberbackward);
    m_oi.getButton(0, Constants.Buttons.B_BUTTON).whileHeld(ClimberLF);
    m_oi.getButton(0, Constants.Buttons.Y_BUTTON).whileHeld(ClimberRF);
    // m_oi.getButton(0, Constants.Buttons.B_BUTTON).whileHeld(turret);
    // m_oi.getButton(0, Constants.Buttons.Y_BUTTON).whileHeld(turretr);
    m_oi.getButton(0, Constants.Buttons.X_BUTTON).whileHeld(ClimberRB);
    m_oi.getButton(0, Constants.Buttons.A_BUTTON).whileHeld(ClimberLB);
    // m_oi.getButton(1, Constants.Buttons.LEFT_BUMPER).whileHeld(LowerHubShooter);
  }

  public Command getAutonomousCommand() {
      // 1: C5, 2: C4, 3: C2, 4: M4, 5: M2
      int num = 0;
      String chosenAuton = (new String[]{"NONE", "3L", "3M", "3S", "2M", "2S"})[num];
    
    RamseteCommand ramseteCommand11 = new RamseteCommand(
        traj11,
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

    RamseteCommand ramseteCommand12 = new RamseteCommand(
        traj12,
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
    RamseteCommand ramseteCommand13 = new RamseteCommand(
        traj13,
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

    RamseteCommand ramseteCommand21 = new RamseteCommand(
        traj21,
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

    RamseteCommand ramseteCommand22 = new RamseteCommand(
        traj22,
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

    RamseteCommand ramseteCommand23 = new RamseteCommand(
        traj23,
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

    RamseteCommand ramseteCommand24 = new RamseteCommand(
        traj24,
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

    RamseteCommand ramseteCommand31 = new RamseteCommand(
        traj31,
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

    RamseteCommand ramseteCommand32 = new RamseteCommand(
        traj32,
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

    RamseteCommand ramseteCommand33 = new RamseteCommand(
        traj33,
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

    RamseteCommand ramseteCommand34 = new RamseteCommand(
        traj34,
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

    if (chosenAuton.equals("3L")) {
      m_drive.resetOdometry(traj31.getInitialPose());
      return new ParallelCommandGroup(nintake, ramseteCommand31).andThen(() -> m_drive.tankDriveVolts(0, 0))
          .andThen(autonVision)
          .andThen(new ParallelCommandGroup(n2intake, ramseteCommand32).andThen(() -> m_drive.tankDriveVolts(0, 0)))
          .andThen(new ParallelCommandGroup(n3intake, ramseteCommand33)).andThen(() -> m_drive.tankDriveVolts(0, 0))
          .andThen(new ParallelCommandGroup(
              new StartEndCommand(() -> m_drive.arcadeDrive(0, 0.4), () -> m_drive.arcadeDrive(0, 0), m_drive)
                  .withTimeout(1.7),
              autonVision2));
    } else if (chosenAuton.equals("3S")) {
      m_drive.resetOdometry(traj31.getInitialPose());
      return new ParallelCommandGroup(nintake, ramseteCommand31).andThen(() -> m_drive.tankDriveVolts(0, 0))
          .andThen(autonVision);
    } else if (chosenAuton.equals("3M")) {
      m_drive.resetOdometry(traj31.getInitialPose());
      return new ParallelCommandGroup(nintake, ramseteCommand31).andThen(() -> m_drive.tankDriveVolts(0, 0))
          .andThen(autonVision)
          .andThen(new ParallelCommandGroup(n2intake, ramseteCommand32).andThen(() -> m_drive.tankDriveVolts(0, 0)))
          .andThen(ramseteCommand34).andThen(autonVision2);
    } else if (chosenAuton.equals("2S")) {
      nintake = new NIntake(m_intake, m_hopper, 2.5, 0);
      m_drive.resetOdometry(traj21.getInitialPose());
      return new ParallelCommandGroup(nintake, ramseteCommand21).andThen(() -> m_drive.tankDriveVolts(0, 0))
          .andThen(autonVision);
    } else if (chosenAuton.equals("2M")) {
      nintake = new NIntake(m_intake, m_hopper, 2.5, 0);
      n2intake = new NIntake(m_intake, m_hopper, 4, 0);
      m_drive.resetOdometry(traj21.getInitialPose());
      return new ParallelCommandGroup(nintake, ramseteCommand21).andThen(() -> m_drive.tankDriveVolts(0, 0))
          .andThen(autonVision)
          .andThen(new ParallelCommandGroup(n2intake, ramseteCommand22))
          .andThen(() -> m_drive.tankDriveVolts(0, 0))
          .andThen(ramseteCommand23)
          .andThen(() -> m_drive.tankDriveVolts(0, 0)).andThen(autonVision2);
    }
    
    //return null;
    // return new ParallelCommandGroup(new
    // StartEndCommand(()->m_drive.arcadeDrive(0,0.4), ()->m_drive.arcadeDrive(0,0),
    // m_drive).withTimeout(1.7), autonVision2);

    // path1 19 inches from right corner, 5 inches from front

    // m_drive.resetOdometry(traj11.getInitialPose());
    // return new
    // ParallelCommandGroup(nintake,ramseteCommand11).andThen(ramseteCommand12.andThen(ramseteCommand33.andThen(()
    // ->m_drive.tankDriveVolts(0, 0))));

    // path2 16 inch left corner
    // m_drive.resetOdometry(traj21.getInitialPose());
    // return new ParallelCommandGroup(nintake, ramseteCommand21).andThen(() ->
    // m_drive.tankDriveVolts(0, 0)).andThen(autonVision)
    // .andThen(new ParallelCommandGroup(n2intake, ramseteCommand22))
    // .andThen(() ->m_drive.tankDriveVolts(0, 0))
    // .andThen(ramseteCommand24).andThen(() ->m_drive.tankDriveVolts(0, 0))
    // .andThen(new ParallelCommandGroup(new
    // StartEndCommand(()->m_drive.arcadeDrive(0,0.45),
    // ()->m_drive.arcadeDrive(0,0), m_drive).withTimeout(1.2), autonVision2));
    // path3
    // return null;

    // return (new ParallelCommandGroup(
    //     new StartEndCommand(() -> m_drive.arcadeDrive(0, 0.4), () -> m_drive.arcadeDrive(0, 0), m_drive)
    //         .withTimeout(1.7),
    //     autonVision2));
    // return null;
    // 2ball path 3
    // m_drive.resetOdometry(traj31.getInitialPose());
    // return new ParallelCommandGroup(nintake, ramseteCommand31).andThen(() ->
    // m_drive.tankDriveVolts(0, 0)).andThen(autonVision);

    // path3 19in from left corner, 4 inch from front bar 39 inches from ball
    // m_drive.resetOdometry(traj31.getInitialPose());
    // return new ParallelCommandGroup(nintake, ramseteCommand31).andThen(() ->
    // m_drive.tankDriveVolts(0, 0)).andThen(autonVision)
    // .andThen(new ParallelCommandGroup(n2intake, ramseteCommand32).andThen(() ->
    // m_drive.tankDriveVolts(0, 0)))
    // .andThen(new ParallelCommandGroup(n3intake, ramseteCommand33)).andThen(() ->
    // m_drive.tankDriveVolts(0, 0)).andThen(new ParallelCommandGroup(new
    // StartEndCommand(()->m_drive.arcadeDrive(0,0.5), ()->m_drive.arcadeDrive(0,0),
    // m_drive).withTimeout(2), autonVision2));

    // return autonVision;

    // New Path #1 using pathplanner, starts in tarmac near acute angle: 5 ball
    // auton
    // m_drive.resetOdometry(aPath11.getTrajectory().getInitialPose());
    // return new ParallelCommandGroup(new ExtendIntake(this),
    //     aPath11.getRameseteCommand(m_drive))
    //     .andThen(()->m_drive.tankDriveVolts(0,0))
    //     .andThen(new VisionAuto(m_LL, m_turret, m_shroud, m_shooter, m_oi, m_feeder, m_hopper,
    //     m_intake, false, 2))
    //     .andThen(new ParallelCommandGroup(new IntakeRetract(this),
    //     aPath12.getRameseteCommand(m_drive)))
    //     .andThen(()->m_drive.tankDriveVolts(0,0))
    //     .andThen(aPath13.getRameseteCommand(m_drive))
    //     .andThen(()->m_drive.tankDriveVolts(0,0))
    //     .andThen(new VisionAuto(m_LL, m_turret, m_shroud, m_shooter, m_oi, m_feeder, m_hopper,
    //     m_intake, false, 4))
    //     .andThen(new ExtendIntake(this))
    //     .andThen(new VisionAuto(m_LL, m_turret, m_shroud, m_shooter, m_oi, m_feeder, m_hopper,
    //     m_intake, false, 5))
    ;

    // New Path #2 using pathplanner, starts in tarmac near obtuse angle: 5 ball
    // auton
    // m_drive.resetOdometry(aPath21.getTrajectory().getInitialPose());
    // return new ParallelCommandGroup(new ExtendIntake(this),
    // aPath21.getRameseteCommand(m_drive))
    // .andThen(()->m_drive.tankDriveVolts(0,0))
    // .andThen(new VisionAuto(this))
    // .andThen(new ParallelCommandGroup(new IntakeRetract(this),
    // aPath22.getRameseteCommand(m_drive)))
    // .andThen(()->m_drive.tankDriveVolts(0,0))
    // .andThen(aPath23.getRameseteCommand(m_drive))
    // .andThen(()->m_drive.tankDriveVolts(0,0))
    // .andThen(new VisionAuto(this))
    // .andThen(new ExtendIntake(this))
    // .andThen(new VisionAuto(this))
    // ;

    // New Path #3 using pathplanner, starts in tarmac near obtuse angle, but more
    // towards center: 4 ball auton
    // m_drive.resetOdometry(aPath31.getTrajectory().getInitialPose());
    // return new ParallelCommandGroup(new ExtendIntake(this),
    // aPath31.getRameseteCommand(m_drive))
    // .andThen(()->m_drive.tankDriveVolts(0,0))
    // .andThen(new VisionAuto(this))
    // .andThen(new ParallelCommandGroup(new IntakeRetract(this),
    // aPath32.getRameseteCommand(m_drive)))
    // .andThen(()->m_drive.tankDriveVolts(0,0))
    // .andThen(aPath33.getRameseteCommand(m_drive))
    // .andThen(new VisionAuto(this))
    // ;

    // New Path #3 using pathplanner, starts in tarmac near obtuse angle, but more
    // towards center: 4 ball auton
    // m_drive.resetOdometry(aPath41.getTrajectory().getInitialPose());
    // return new ParallelCommandGroup(new ExtendIntake(this),
    // aPath41.getRameseteCommand(m_drive))
    // .andThen(()->m_drive.tankDriveVolts(0,0))
    // .andThen(new VisionAuto(this))
    // .andThen(aPath42.getRameseteCommand(m_drive))
    // .andThen(new OutTakeCommand(this));

    // test
    // return ramseteCommand11;
    return null;
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
