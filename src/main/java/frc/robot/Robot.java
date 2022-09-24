// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.

    CameraServer.startAutomaticCapture();
    m_robotContainer = new RobotContainer();
    m_robotContainer.m_drive.BackRight.setNeutralMode(NeutralMode.Coast);
    m_robotContainer.m_drive.BackLeft.setNeutralMode(NeutralMode.Coast);
    m_robotContainer.m_drive.FrontRight.setNeutralMode(NeutralMode.Coast);
    m_robotContainer.m_drive.FrontLeft.setNeutralMode(NeutralMode.Coast);
    // m_robotContainer.m_turret.turretMotor.setNeutralMode(NeutralMode.Coast);
  }


  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Turret Encoder", m_robotContainer.m_turret.getTurretRot());
    // SmartDashboard.putNumber("leftside", Constants.leftRate);
    // SmartDashboard.putNumber("rightside", Constants.rightRate);
    // SmartDashboard.putBoolean("MovingShoot", Constants.moveshoot);
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    SmartDashboard.putNumber("Heading", m_robotContainer.m_drive.getHeading());
    SmartDashboard.putNumber("TurretPos", m_robotContainer.m_turret.getTurretRot());
    SmartDashboard.putNumber("X Dis", m_robotContainer.m_drive.getXOffset());
    SmartDashboard.putNumber("Y Dis", m_robotContainer.m_drive.getYOffset());
    SmartDashboard.putNumber("DegToTurn", m_robotContainer.m_drive.degToTurn());
    SmartDashboard.putBoolean("ButtonPushed", m_robotContainer.m_oi.getButton(0, Constants.Buttons.RIGHT_BUMPER).get());
    SmartDashboard.putNumber("Shroud", m_robotContainer.m_shroud.getShroudPosition());
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.m_drive.BackRight.setNeutralMode(NeutralMode.Coast);
    m_robotContainer.m_drive.BackLeft.setNeutralMode(NeutralMode.Coast);
    m_robotContainer.m_drive.FrontRight.setNeutralMode(NeutralMode.Coast);
    m_robotContainer.m_drive.FrontLeft.setNeutralMode(NeutralMode.Coast);
    // m_robotContainer.m_turret.turretMotor.setNeutralMode(NeutralMode.Coast);
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_robotContainer.m_shroud.resetEncoder();
    m_robotContainer.m_drive.resetEncoders();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    m_robotContainer.m_drive.BackRight.setNeutralMode(NeutralMode.Brake);
    m_robotContainer.m_drive.BackLeft.setNeutralMode(NeutralMode.Brake);
    m_robotContainer.m_drive.FrontRight.setNeutralMode(NeutralMode.Brake);
    m_robotContainer.m_drive.FrontLeft.setNeutralMode(NeutralMode.Brake);
    // m_robotContainer.m_turret.turretMotor.setNeutralMode(NeutralMode.Brake);
    // schedule the autonomous command (example)
    m_robotContainer.m_shroud.resetEncoder();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // m_robotContainer.m_shroud.resetEncoder();
    m_robotContainer.m_climber.extendPistons();
    m_robotContainer.m_drive.BackRight.setNeutralMode(NeutralMode.Brake);
    m_robotContainer.m_drive.BackLeft.setNeutralMode(NeutralMode.Brake);
    m_robotContainer.m_drive.FrontRight.setNeutralMode(NeutralMode.Brake);
    m_robotContainer.m_drive.FrontLeft.setNeutralMode(NeutralMode.Brake);
    // m_robotContainer.m_turret.turretMotor.setNeutralMode(NeutralMode.Brake);
    Constants.climbing = false;
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.m_drive.getNavx().resetDisplacement();
    m_robotContainer.m_drive.resetEncoders();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    // SmartDashboard.putNumber("leftEncoder", m_robotContainer.m_drive.getLeftEncoderPosition());
    // SmartDashboard.putNumber("rightEncoder", m_robotContainer.m_drive.getRightEncoderPosition());
    // SmartDashboard.putNumber("X displacement", m_robotContainer.m_drive.getNavx().getDisplacementX());
    // SmartDashboard.putNumber("Z displacement", m_robotContainer.m_drive.getNavx().getDisplacementZ());
    // SmartDashboard.putNumber("Shroud Encoder", m_robotContainer.m_shroud.getShroudPosition());
    // SmartDashboard.putNumber("TurretEncoder", m_robotContainer.m_turret.getTurretRot());
    // SmartDashboard.putNumber("ShooterRate", m_robotContainer.m_shooter.getRate());
    // SmartDashboard.putNumber("LLYaw", m_robotContainer.m_LL.LLTable.getEntry("tx").getDouble(0));
    // SmartDashboard.putNumber("LLP-itch", m_robotContainer.m_LL.getAngleY());
    // SmartDashboard.putBoolean("TargetFound", m_robotContainer.m_LL.targetVisible());
    // SmartDashboard.putNumber("TurretRotation", m_robotContainer.m_turret.getTurretRot());
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
