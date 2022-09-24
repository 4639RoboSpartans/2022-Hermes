package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

import static frc.robot.Constants.Axes;

public class DriveCommand extends CommandBase {

    private DriveSubsystem m_drive;
    private OI oi;

    private final SlewRateLimiter speedLimiter = new SlewRateLimiter(12);//8
	private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(5);

    public DriveCommand(DriveSubsystem m_drive, OI oi) {
        this.m_drive = m_drive;
        this.oi = oi;
        
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_drive.arcadeDrive(0, 0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Constants.leftRate = m_drive.getLeftEncoderRate();
        Constants.rightRate = m_drive.getRightEncoderRate();
        
        
        m_drive.arcadeDrive(
            speedLimiter.calculate(-oi.getAxis(0, Axes.LEFT_STICK_Y)) *Constants.ROTATION_SENSITIVITY,
			rotationLimiter.calculate(-oi.getAxis(0, Axes.RIGHT_STICK_X))
						* Constants.DRIVE_SPEED);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
