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
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class RobotContainer {
  public DriveSubsystem m_drive = new DriveSubsystem();
  public IntakeSubsystem m_intake = new IntakeSubsystem();
  public OI m_oi = new OI();

  public RobotContainer() {

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    m_drive.setDefaultCommand(new DriveCommand(m_drive, m_oi));
    m_oi.getPovButton(1, 0).whileHeld(new RunCommand(() -> m_intake.extendPistons(), m_intake));
    m_oi.getPovButton(1, 180).whileHeld(new RunCommand(() -> m_intake.retractPistons(), m_intake));
    m_oi.getButton(1, Constants.Buttons.X_BUTTON).whileHeld(new IntakeCommand());
    m_oi.getButton(1, Constants.Buttons.A_BUTTON).whileHeld(new OutTakeCommand());
    m_oi.getButton(1, Constants.Buttons.LEFT_BUMPER).whileHeld(new VisionAimCommand());
    m_oi.getButton(1, Constants.Buttons.RIGHT_BUMPER).whileHeld(new ShooterCommand());
    m_oi.getButton(1, Constants.Buttons.Y_BUTTON).whileHeld(new ClimberForwardCommand());
    m_oi.getButton(1, Constants.Buttons.B_BUTTON).whileHeld(new ClimberBackwardCommand());
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
