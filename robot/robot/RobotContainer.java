// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ClimberCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.List;
/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  //private final ADXRS450_Gyro  m_gyro = new ADXRS450_Gyro();

  // The driver's controller
  Joystick m_driverJoystick = new Joystick(OIConstants.kDriverJoystickPort);
  Joystick m_driverJoystickTurning = new Joystick(OIConstants.kDriverJoystickTurningPort);
  XboxController m_secondaryController = new XboxController(OIConstants.kSecondaryControllerPort);

  private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_driverJoystickTurning);

  //ClimberSubsystem climbersubsystem = new ClimberSubsystem();
  //ClimberCommand climbercommand = new ClimberCommand(climbersubsystem, m_secondaryController);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  public RobotContainer() {
    //sendable chooser

    m_chooser.addOption("Speaker Only VVV", null);

    m_chooser.setDefaultOption("BLUE_OnlySpeakerStartingNote", "BLUE_OnlySpeakerStartingNote");
    m_chooser.addOption("BLUE_OnlySpeakerLeftNote", "BLUE_OnlySpeakerLeftNote");
    m_chooser.addOption("BLUE_OnlySpeakerTwoLeftNotes", "BLUE_OnlySpeakerTwoLeftNotes");
    m_chooser.addOption("BLUE_OnlySpeakerRightNote", "BLUE_OnlySpeakerRightNote");
    m_chooser.addOption("BLUE_OnlySpeakerTwoRightNotes", "BLUE_OnlySpeakerTwoRightNotes");
    m_chooser.addOption("BLUE_OnlySpeakerCenterNote", "BLUE_OnlySpeakerCenterNote");
    m_chooser.addOption("BLUE_OnlySpeakerLeftCenterNotes", "BLUE_OnlySpeakerLeftCenterNotes");
    m_chooser.addOption("BLUE_OnlySpeakerRightCenterNotes", "BLUE_OnlySpeakerRightCenterNotes");

    m_chooser.addOption("AMP Only VVV", null);

    m_chooser.addOption("BLUE_OnlyAmpStartingNote", "BLUE_OnlyAmpStartingNote");
    m_chooser.addOption("BLUE_OnlyAmpLeftNote", "BLUE_OnlyAmpLeftNote");

    SmartDashboard.putData("Autonomous Chooser", m_chooser);

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(Math.pow(m_driverJoystick.getY(), 3), Math.pow(OIConstants.kDriveDeadband, 3)),
                -MathUtil.applyDeadband(Math.pow(m_driverJoystick.getX(), 3), Math.pow(OIConstants.kDriveDeadband, 3)),
                -MathUtil.applyDeadband(Math.pow(m_driverJoystick.getZ(), 3), Math.pow(OIConstants.kDriveDeadband, 3)),
                true, true),
            m_robotDrive));
  
        //climbersubsystem.setDefaultCommand(climbercommand);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    //new JoystickButton(m_driverController, Button.kR1.value)
    //    .whileTrue(new RunCommand(
    //        () -> m_robotDrive.setX(),
    //        m_robotDrive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
  TrajectoryConfig config =
      new TrajectoryConfig(
              AutoConstants.kMaxSpeedMetersPerSecond,
              AutoConstants.kMaxAccelerationMetersPerSecondSquared)
          // Add kinematics to ensure max speed is actually obeyed
          .setKinematics(DriveConstants.kDriveKinematics);


  var thetaController =
    new ProfiledPIDController(
      AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
  thetaController.enableContinuousInput(-Math.PI, Math.PI);

  System.out.println(m_robotDrive.getPose());




  //TRAJECTORIES FOR EVERY AUTONOMOUS PROCEDURE
  //BLUE SIDE AUTONOMOUS:
  Trajectory BLUE_LeftNote =
  TrajectoryGenerator.generateTrajectory(
      m_robotDrive.getPose(),
      List.of(),
      new Pose2d(Units.inchesToMeters(114), Units.inchesToMeters(161.64), new Rotation2d(0)),
      config);

  Trajectory BLUE_RightNote =
  TrajectoryGenerator.generateTrajectory(
      m_robotDrive.getPose(),
      List.of(),
      new Pose2d(Units.inchesToMeters(114), Units.inchesToMeters(47.64), new Rotation2d(0)),
      config);

  Trajectory Blue_MiddleNote =
  TrajectoryGenerator.generateTrajectory(
      m_robotDrive.getPose(),
      List.of(),
      new Pose2d(Units.inchesToMeters(114), Units.inchesToMeters(104.64), new Rotation2d(0)),
      config);

  Trajectory BLUE_Speaker =
  TrajectoryGenerator.generateTrajectory(
      m_robotDrive.getPose(),
      List.of(),
      new Pose2d(Units.inchesToMeters((Constants.DriveConstants.kWheelBase)/2), Units.inchesToMeters(104.64), new Rotation2d(0)),
      config);

  Trajectory BLUE_Amp =
  TrajectoryGenerator.generateTrajectory(
      m_robotDrive.getPose(),
      List.of(),
      new Pose2d(Units.inchesToMeters(76.1), Units.inchesToMeters((Constants.DriveConstants.kWheelBase)/2), new Rotation2d(Units.radiansToDegrees(90))),
      config);

  SwerveControllerCommand BLUE_LeftNoteCommand =
  new SwerveControllerCommand(
      BLUE_LeftNote,
      m_robotDrive::getPose, // Functional interface to feed supplier
      DriveConstants.kDriveKinematics,
      // Position controllers
      new PIDController(AutoConstants.kPXController, 0, 0),
      new PIDController(AutoConstants.kPYController, 0, 0),
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);
    
  SwerveControllerCommand BLUE_RightNoteCommand =
  new SwerveControllerCommand(
      BLUE_RightNote,
      m_robotDrive::getPose, // Functional interface to feed supplier
      DriveConstants.kDriveKinematics,
      // Position controllers
      new PIDController(AutoConstants.kPXController, 0, 0),
      new PIDController(AutoConstants.kPYController, 0, 0),
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);

  SwerveControllerCommand BLUE_MiddleNoteCommand =
  new SwerveControllerCommand(
      Blue_MiddleNote,
      m_robotDrive::getPose, // Functional interface to feed supplier
      DriveConstants.kDriveKinematics,
      // Position controllers
      new PIDController(AutoConstants.kPXController, 0, 0),
      new PIDController(AutoConstants.kPYController, 0, 0),
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);

  SwerveControllerCommand BLUE_SpeakerCommand =
  new SwerveControllerCommand(
      BLUE_Speaker,
      m_robotDrive::getPose, // Functional interface to feed supplier
      DriveConstants.kDriveKinematics,
      // Position controllers
      new PIDController(AutoConstants.kPXController, 0, 0),
      new PIDController(AutoConstants.kPYController, 0, 0),
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);

  SwerveControllerCommand BLUE_AmpCommand =
  new SwerveControllerCommand(
      BLUE_Amp,
      m_robotDrive::getPose, // Functional interface to feed supplier
      DriveConstants.kDriveKinematics,
      // Position controllers
      new PIDController(AutoConstants.kPXController, 0, 0),
      new PIDController(AutoConstants.kPYController, 0, 0),
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive);

    //the autonomous
    if (m_chooser.getSelected() == "BLUE_OnlySpeakerStartingNote") {
            return Commands.sequence(
            BLUE_SpeakerCommand,
            //shoot after the InstantCommand
            new InstantCommand(() -> m_robotDrive.drive(0,0,0,false,true)));
    } 
    if (m_chooser.getSelected() == "BLUE_OnlySpeakerLeftNote") {
            return Commands.sequence(
            BLUE_SpeakerCommand,
            //shoot
            //lower intake (which would turn on intake motor also)
            BLUE_LeftNoteCommand,
            //raise intake
            BLUE_SpeakerCommand,
            //shoot after the InstantCommand
            new InstantCommand(() -> m_robotDrive.drive(0,0,0,false,true)));
    } 
    if (m_chooser.getSelected() == "BLUE_OnlyAmpStartingNote") {
            return Commands.sequence(
            BLUE_AmpCommand,
            //shoot after the InstantCommand
            new InstantCommand(() -> m_robotDrive.drive(0,0,0,false,true)));
    } 
    if (m_chooser.getSelected() == "BLUE_OnlyAmpLeftNote") {
            return Commands.sequence(
            BLUE_AmpCommand,
            //shoot
            //lower intake
            BLUE_LeftNoteCommand,
            //raise intake
            BLUE_AmpCommand,
            //shoot after instant command
            new InstantCommand(() -> m_robotDrive.drive(0,0,0,false,true)));
    } 
    return null;
  }
}
