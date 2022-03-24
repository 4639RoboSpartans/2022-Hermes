// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	public static final int DriveMotorFrontLeft = 1;
	public static final int DriveMotorBackLeft=2;
	public static final int DriveMotorFrontRight=3;
	public static final int DriveMotorBackRight=4;
	public static final int IntakeMotor = 9;
	public static final int HopperMotor=8;
	public static final int FeedMotor=7;
	public static final int TurretMotor=10;	//Need to update this value
	public static final int ShooterMotor=6;	//Need to update this value
	public static final int ShroudMotor=5;	//Need to update this value
	public static final int LClimberMotor=11;
	public static final int RClimberMotor=12;

	public static final int ForwardPiston= 1;
	public static final int BackwardPiston =2;

	public static final int feederSensor=1;

	//Subject to change
	public static final double ksVolts=0.69874;//0.66099;
	public static final double kvVoltSecondsPerMeter = 0.012955;//0.026728;
    public static final double kaVoltSecondsSquaredPerMeter = 0.0028624;//0.0028603;
	public static final double kPDriveVel = 0.12992;
	//Subject to change

	public static final double kTrackwidthMeters = 0.665;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);
	public static final double kMaxSpeedMetersPerSecond = 3.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3.5;
	public static final double kRamseteB = 15;//2
    public static final double kRamseteZeta = 4;//0.7

public static boolean pushballs = false;



    public static final double DEADZONE_VALUE = 0.01;
	public static final int NUMBER_OF_CONTROLLERS = 2;

	public static final double ROTATION_SENSITIVITY = 0.7;//0.75
    
    public enum Axes {
		LEFT_STICK_X(0), LEFT_STICK_Y(4), LEFT_TRIGGER(2), RIGHT_TRIGGER(3), RIGHT_STICK_X(1), RIGHT_STICK_Y(5);

		private final int value;

		Axes(int value) {
			this.value = value;
		}

		public int getValue() {
			return value;
		}
	}

	public enum Buttons {
		A_BUTTON(1), B_BUTTON(2), X_BUTTON(3), Y_BUTTON(4), LEFT_BUMPER(5), RIGHT_BUMPER(6), BACK_BUTTON(
				7), START_BUTTON(8), LEFT_STICK(9), RIGHT_STICK(10);

		private final int value;

		private Buttons(int value) {
			this.value = value;
		}

		public int getValue() {
			return value;
		}

	}   
}
