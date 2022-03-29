package frc.robot.AutonCommands;

import frc.robot.Constants;
import frc.robot.AutonCommands.ExtendIntake;
import frc.robot.AutonCommands.ExtendIntakeRetract;
import frc.robot.AutonCommands.IntakeRetract;
import frc.robot.AutonCommands.ShooterAuto;
import frc.robot.AutonCommands.VisionAuto;
import frc.robot.commands.ClimberBackwardCommand;
import frc.robot.commands.ClimberForwardCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.OutTakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.TurretCommand;
import frc.robot.commands.TurretCommandR;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

public class AutonPath {
    Trajectory trajectory = new Trajectory();
    public AutonPath(String filePath){
        try{
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filePath);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        }
        catch(IOException e){
            DriverStation.reportError("Unable to open trajectory: " + filePath, e.getStackTrace());
        }
    }

    public RamseteCommand getRameseteCommand(DriveSubsystem m_drive){
        return new RamseteCommand(
            trajectory,
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
    }

    public Trajectory getTrajectory() {
        return trajectory;
    }
}
