// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	public static class OperatorConstants {
		public static final int kDriverControllerPort = 0;
	}

	public static class CANids {
		// SPARK MAX CAN IDs
		public static final int kFrontLeftDrivingCanId = 1;
		public static final int kRearLeftDrivingCanId = 5;
		public static final int kFrontRightDrivingCanId = 7;
		public static final int kRearRightDrivingCanId = 3;

		public static final int kFrontLeftTurningCanId = 2;
		public static final int kRearLeftTurningCanId = 6;
		public static final int kFrontRightTurningCanId = 8;
		public static final int kRearRightTurningCanId = 4;

		public static final int kRightElevatorCanId = 10;
		public static final int kLeftElevatorCanId = 11;

		public static final int KOutakeCanId = 12;

	}

	public static class DriveConstants {
		public static class Mode {
			public static final double Normal = 1;
			public static final double Turbo = 1.5;
			public static final double Precision = 0.5;
			public static final double SuperPrecision = 0.3;
		}

		public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
		public static final double kFrontRightChassisAngularOffset = 0;
		public static final double kBackLeftChassisAngularOffset = Math.PI;
		public static final double kBackRightChassisAngularOffset = Math.PI / 2;
	}

	public static class robot {
		public static final PIDConstants kPIDDrive = new PIDConstants(3,1,0);
		public static final PIDConstants kPIDTurning = new PIDConstants(robot.kPTurning,robot.kITurning,robot.kDTurning);
		// public static final PIDController kPIDTurningController = new PIDController(robot.kPTurning,robot.kITurning,robot.kDTurning);
		// public static final PIDController kPIDDriveController = new PIDController(1, 0, 0);
		// public static final PIDController kPIDElevator = new PIDController(1,0,0);

		public static final double kPTurning = 0.6;
		public static final double kITurning = 0;
		public static final double kDTurning = 0;

		public static final double kPhysicalMaxSpeedMetersPerSecond = 4.7;
		public static final double kTeleDriveAccelerationUnitsPerSecond = 8;
		public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2*2*Math.PI;

		public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
		public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;

		public static final double kModuleMaxAngularVelocity = Math.PI / 8;
		public static final double kModuleMaxAngularAcceleration = 2 * Math.PI / 8;

		public static final double kTrackWidth = 0.65;
		// Distance between right and left wheels
		public static final double kWheelBase = 0.65;
		// Distance between front and back wheels
		public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
				new Translation2d(kWheelBase / 2, kTrackWidth / 2),
				new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
				new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
				new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));


	}

	public static class xboxConstants {
		public static final double kDeadband = 0.15;
		public static final int kDriverControllerPort = 0;
		public static final int kOperatorControllerPort = 1;

		public static final int kDriverYAxis = 1;
		public static final int kDriverXAxis = 0;
		public static final int kDriverRotAxis = 4;
		public static final int kDriverFieldOrientedButtonIdx = 1;

		public static class buttons {
			public static final int upLeft = 1;
			public static final int downLeft = 5;
			public static final int upRight = 2;
			public static final int downRight = 6;
			public static final int l3 = 9;
			public static final int r3 = 10;
		}
	}

	public static class visionConstants {
		public static final String kCameraName = "Camera_Module_v2";

		public static final double kCameraHeightMeters = 0.1;
		public static final double kCameraPitchRadians = Math.PI / 2;

		public static final double kTargetHeightMeters = 0.3;
		public static final double kTargetPitchRadians = Math.PI / 2;
		public static final Transform3d cameraToRobot = new Transform3d(0.2, 0.05, 0, new Rotation3d());
	}
	
	public static class elevatorConstants {

		// precision mode will be enforced on drive if the elevator is beyond this point
		public static final double kForcedPrecision = 5;
		public static class setPoints {
			public static final double level0 = 0;
			public static final double level1 = 2.6;
			public static final double level2 = 4.4;
			public static final double level3 = 6.6;
			public static final double maxSafety = 6.72;
		}
		
		public static final double kMaxSpeed = 8;
		public static final double kMaxAcceleration = 5;

		public static class PID {
			public static final double kP = 1;  	// hran
			public static final double kI = 0.2;   // hran
			public static final double kD = 0;    // hran


			public static final TrapezoidProfile.Constraints kelevatorConstraints = 
				new TrapezoidProfile.Constraints(kMaxSpeed, kMaxAcceleration);
		}

		public static class FeedForward {
			public static final double kS = 0.675;
			public static final double kG = 1.35;
			public static final double kV = 0.6;
		}

	}

	public static class OutakeConstants {
		public static final double kOutakeVoltage = 2;
		public static final double kEmergencyVoltage = 3;
		public static final double kReverseVoltage = -1.3;
		public static final double kSlowVoltage = 1;
	}

}