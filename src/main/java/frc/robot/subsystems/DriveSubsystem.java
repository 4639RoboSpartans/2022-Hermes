package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
    public WPI_TalonFX FrontLeft = new WPI_TalonFX(Constants.DriveMotorFrontLeft);
    public WPI_TalonFX BackLeft = new WPI_TalonFX(Constants.DriveMotorBackLeft);
    public WPI_TalonFX FrontRight = new WPI_TalonFX(Constants.DriveMotorFrontRight);
    public WPI_TalonFX BackRight = new WPI_TalonFX(Constants.DriveMotorBackRight);
    private AHRS navx = new AHRS();// 0.665 m track width
    private DifferentialDrive m_drive;
    private DifferentialDriveOdometry m_odometry;

    public DriveSubsystem() {
        FrontLeft.configFactoryDefault();
        BackLeft.configFactoryDefault();
        FrontRight.configFactoryDefault();
        BackRight.configFactoryDefault();

        FrontLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        BackLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        FrontRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        BackRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        FrontLeft.setNeutralMode(NeutralMode.Brake);
        BackLeft.setNeutralMode(NeutralMode.Brake);
        FrontRight.setNeutralMode(NeutralMode.Brake);
        BackRight.setNeutralMode(NeutralMode.Brake);

        BackLeft.follow(FrontLeft);
        BackRight.follow(FrontRight);

        
        m_drive = new DifferentialDrive(FrontLeft, FrontRight);
        m_drive.setSafetyEnabled(false);

        m_odometry = new DifferentialDriveOdometry(navx.getRotation2d());
        navx.setAngleAdjustment(-140);

    }

    //8.2,4.2
    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        m_odometry.update(
                navx.getRotation2d(), getLeftEncoderPosition(), getRightEncoderPosition());
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftEncoderRate(), getRightEncoderRate());
    }
    public double getXOffset(){
        return getPose().getX()-4.2;
    }
    public double getYOffset(){
        return getPose().getY()-8.2;
    }
    public double degToTurn(){
        if(getXOffset()>0&&getYOffset()>0){
            return 180+Math.toDegrees(Math.atan(getYOffset()/getXOffset()));
        }else if(getXOffset()<0&&getYOffset()>0){
            return 360+Math.toDegrees(Math.atan(getYOffset()/getXOffset()));
        }else if(getXOffset()<0&&getYOffset()<0){
            return Math.toDegrees(Math.atan(getYOffset()/getXOffset()));
        }else if(getXOffset()>0&&getYOffset()<0){
            return 180-Math.toDegrees(Math.atan(getYOffset()/getXOffset()));
        }
        return 0;
    }

    public void resetEncoders(){
        FrontLeft.setSelectedSensorPosition(0);
        FrontRight.setSelectedSensorPosition(0);
        BackLeft.setSelectedSensorPosition(0);
        BackRight.setSelectedSensorPosition(0);
    }
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(pose, navx.getRotation2d());
        
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        FrontLeft.setVoltage(leftVolts);
        FrontRight.setVoltage(-rightVolts);
        m_drive.feed();
    }
 

    public double getAverageEncoderDistance() {
        return (getLeftEncoderPosition() + (getRightEncoderPosition())) / 2.0;
    }

    public void setMaxOutput(double maxOutput) {
        m_drive.setMaxOutput(maxOutput);
    }

    public void zeroHeading() {
        navx.reset();
    }

    public double getHeading() {
        if(navx.getRotation2d().getDegrees()%360<0){
            return (navx.getRotation2d().getDegrees()%360)+360;
        }
        return navx.getRotation2d().getDegrees()%360;
    }

    public double getTurnRate() {
        return -navx.getRate();
    }

    public AHRS getNavx() {
        return navx;
    }

    public double getXDisplacement() {
        return navx.getDisplacementX();
    }

    public double getYDisplacement() {
        return navx.getDisplacementY();
    }

    public void arcadeDrive(double speed, double rotation) {
        m_drive.arcadeDrive(speed, rotation);
    }
    //change distance per rotation for more accurate path.
    // 13 to 50, 24 to 50
    public double getLeftEncoderPosition() {
        return ((FrontLeft.getSelectedSensorPosition() + BackLeft.getSelectedSensorPosition()) / 2)*(0.4788/2048)*(312.0/2500);
    }

    public double getRightEncoderPosition() {
        return -((FrontRight.getSelectedSensorPosition() + BackRight.getSelectedSensorPosition()) / 2)*(0.4788/2048)*(312.0/2500);
    }

    public double getLeftEncoderRate() {
        return ((FrontLeft.getSelectedSensorVelocity() + BackLeft.getSelectedSensorVelocity()) / 2)*(0.4788/2048)*(312.0/2500);
    }

    public double getRightEncoderRate() {
        return -((FrontRight.getSelectedSensorVelocity() + BackRight.getSelectedSensorVelocity()) / 2)*(0.4788/2048)*(312.0/2500);
    }

    public void stop() {
        m_drive.stopMotor();
    }

}
