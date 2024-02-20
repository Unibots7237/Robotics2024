// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(27.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(29);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 2;
    public static final int kRearLeftDrivingCanId = 4;
    public static final int kFrontRightDrivingCanId = 8; 
    public static final int kRearRightDrivingCanId = 6;

    public static final int kFrontLeftTurningCanId = 3; 
    public static final int kRearLeftTurningCanId = 5;
    public static final int kFrontRightTurningCanId = 1;
    public static final int kRearRightTurningCanId = 7;

    public static final int rightHangarCanId = 11;
    public static final int leftHangarCanId = 12;

    
    
    public static final boolean kGyroReversed = false;

    public static final double climberArmSpeed = 0.3;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverJoystickPort = 0;
    public static final int kDriverJoystickTurningPort = 1;
    public static final int kSecondaryControllerPort = 2;
    public static final double kDriveDeadband = 0.1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    public static final double stageTagHeight = 48.81;
    public static final double ampTagHeight = 50.13;
    public static final double speakerTagHeight = 53.88; // there are two apriltags below the speaker, one is centered and the other is to its left.
    public static final double distanceBetweenRightEdgesOfSpeakerTags = 22.25; // they are both on the same height. this variable gives the distance between their right edges
    public static final double distanceBetweenSpeakerTags = 15.75; // this is the shortest distance between them

    public static final double BlueSourceRightID = 1;
    public static final double BlueSourceLeftID = 2;

    public static final double RedSourceRightID = 9;
    public static final double ResourceLeftID = 10;

    public static final double BlueAmpID = 6;
    public static final double RedAmpID = 5;

    public static final double BlueSpeakerCenterID = 8;
    public static final double BlueSpeakerRightID = 7;
    
    public static final double RedSpeakerCenterID = 10;
    public static final double RedSpeakerRightID = 9;

    public static final double RedStageSide1ID = 11;
    public static final double RedStageSide2ID = 12;
    public static final double RedStageSide3ID = 13;

    public static final double BlueStageSide1ID = 14;
    public static final double BlueStageSide2ID = 15;
    public static final double BlueStageSide3ID = 16;

    public static final double limelightHeight = 14.5;
    public static final double limelightAngle = 0;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static class Intake {
    // Motors
    public static final int kIntakeMotorId = 9;
    public static final int kPivotMotorId = 10;

    // DIO
    public static final int k_pivotEncoderId = 0;
    public static final int k_intakeLimitSwitchId = 2;

    // Absolute encoder offset
    public static final double k_pivotEncoderOffset = 0.166842; // Straight up, sketchy to reset to "up"

    // Pivot set point angles
    public static final double k_pivotAngleGround = 0;
    public static final double k_pivotAngleSource = 130;
    public static final double k_pivotAngleAmp = k_pivotAngleSource;
    public static final double k_pivotAngleStow = 215;

    // Intake speeds
    public static final double k_intakeSpeed = 0.7;
    public static final double k_ejectSpeed = -0.45;
    public static final double k_feedShooterSpeed = -0.5;
  }

    // Shooter
    public static final int kShooterLeftMotorId = 14;
    public static final int kShooterRightMotorId = 13;
  
    public static final double kShooterP = 0.00005;
    public static final double kShooterI = 0.0;
    public static final double kShooterD = 0.0;
    public static final double kShooterFF = 0.0002;
  
    public static final double kShooterMinOutput = 0;
    public static final double kShooterMaxOutput = 1;

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
} 