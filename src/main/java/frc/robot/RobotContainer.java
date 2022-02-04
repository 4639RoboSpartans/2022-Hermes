// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class RobotContainer {
  public DriveSubsystem m_drive = new DriveSubsystem();
  public OI m_oi = new OI();

  public RobotContainer() {
    
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    m_drive.setDefaultCommand(new DriveCommand(m_drive,m_oi));
  }


  public Command getAutonomousCommand() {
    return null;
  }
}
