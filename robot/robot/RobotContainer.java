// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.hal.AllianceStationID;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.autocommands.IntakeAmp;
import frc.robot.autocommands.IntakeEject;
import frc.robot.autocommands.IntakeGround;
import frc.robot.autocommands.IntakeShooter;
import frc.robot.autocommands.IntakeStow;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.NoCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem.PivotTarget;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Stream;
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

  public final DriveSubsystem m_robotDrive = new DriveSubsystem(m_driverJoystickTurning);
  public final IntakeSubsystem m_intake = IntakeSubsystem.getInstance();
  public final ShooterSubsystem m_shooter = new ShooterSubsystem();
  public final ClimberSubsystem m_climber = new ClimberSubsystem();

  private final IntakeCommand m_intakeCommand = new IntakeCommand(m_intake, m_driverJoystick, m_secondaryController);
  private final NoCommand nocommand = new NoCommand();
  private final ClimberCommand climbercommand = new ClimberCommand(m_climber, m_secondaryController);
  private final ShooterCommand shootercommand = new ShooterCommand(m_shooter);

  //ClimberSubsystem climbersubsystem = new ClimberSubsystem();
  //ClimberCommand climbercommand = new ClimberCommand(climbersubsystem, m_secondaryController);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private final SendableChooser<String> firstChooser = new SendableChooser<>();
  private final SendableChooser<String> secondChooser = new SendableChooser<>();
  private final SendableChooser<String> thirdChooser = new SendableChooser<>();
  private final SendableChooser<String>[] chooserArray = new SendableChooser[10];
  private final Stream<SendableChooser<String>> chooserStream = Arrays.stream(chooserArray);


  // 'top' is with respect to the blue 0,0 corner where the blue and red amp are at the top and the note droppy thing is on the bottom, blue side is on the left
  //i think, need to check what is the origin of the botpose

  //all these values are assumign the robot is on the blue team. there will be a check if statement to replace them to valuess if on red team (only changing the X value really)
  private double noteX = 114;;
  private double leftNoteY = 275.64;
  private double middleNoteY = 218.64;
  private double rightNoteY = 161.64;

  //in inches to be converted to meters Later.
  private double bumperthickness = 3.5;

  private double speakerY = 218.64;
  private double speakerX = (Constants.DriveConstants.kWheelBase)/2 + bumperthickness;

  //keep in mind for the amp the robot also has to turn 90 degrees
  //the angle for the rest of the paths should theoretically be calculatable using trigonoemtry but i need to do that, right now they just face 0 when going (WHICH WILL NOT WORK)
  private double ampY = 323.28-((Constants.DriveConstants.kWheelBase)/2 + bumperthickness);
  private double ampX = 76.1;

  //dumping the red team X values here so i can add the logic later
  //the length of the field in inches is 653.2
  private final double rednoteX = 539.2;
  private final double redspeakerX = 653.2-speakerX;
  private final double redampX = 653.2-76.1;
  
  public static double getRobotDegreeToTarget(double deltaY, double deltaX) {
    if (deltaY >= 0) {
        return Math.abs(Math.toDegrees(Math.atan(deltaY/deltaX)));
    } else {
        return -Math.abs(Math.toDegrees(Math.atan(deltaY/deltaX)));
    }
  }

  public RobotContainer() {

    if (DriverStation.getAlliance().get() == Alliance.Red) {
        noteX = rednoteX;
        speakerX = redspeakerX;
        ampX = redampX;
    }


    //sendable chooser

    SmartDashboard.putData("first autonomous", firstChooser);
    SmartDashboard.putData("second autonomous", secondChooser);

    // Configure the button bindings
    configureButtonBindings();
    
    m_intake.setDefaultCommand(m_intakeCommand);
    m_climber.setDefaultCommand(climbercommand);
    //m_shooter.setDefaultCommand(shootercommand);


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
  
  private final IntakeAmp intakeamp = new IntakeAmp(m_intake);
  private final IntakeEject intakeeject = new IntakeEject(m_intake);
  private final IntakeGround intakeground = new IntakeGround(m_intake);
  private final IntakeStow intakestow = new IntakeStow(m_intake);
  private final IntakeShooter intakeshoot = new IntakeShooter(m_intake);

  private final Command chosenautonomouscommand(String selected) {
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

    if (selected == "intake eject") {
        return intakeeject;
    }
    if (selected == "intake shooter feed") {
        return intakeshoot;
    }
    if (selected == "intake to ground") {
        return intakeground;
    }
    if (selected == "intake to stow") {
        return intakestow;
    }
    if (selected == "intake to amp") {
        return intakeamp;
    }
    if (selected == "to left note") {
        Trajectory LeftNote =
        TrajectoryGenerator.generateTrajectory(
            m_robotDrive.getPose(),
            List.of(),
            new Pose2d(Units.inchesToMeters(noteX), Units.inchesToMeters(leftNoteY), 
                new Rotation2d(getRobotDegreeToTarget((m_robotDrive.getPose().getY() - leftNoteY), (m_robotDrive.getPose().getX() - noteX)))),
            config);

        SwerveControllerCommand LeftNoteCommand =
        new SwerveControllerCommand(
            LeftNote,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,
            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);
        
            return LeftNoteCommand;
    }
    if (selected == "to right note") {
        Trajectory RightNote =
        TrajectoryGenerator.generateTrajectory(
            m_robotDrive.getPose(),
            List.of(),
            new Pose2d(Units.inchesToMeters(noteX), Units.inchesToMeters(rightNoteY), 
                new Rotation2d(getRobotDegreeToTarget((m_robotDrive.getPose().getY() - rightNoteY), (m_robotDrive.getPose().getX() - noteX)))),
            config);

        SwerveControllerCommand RightNoteCommand =
        new SwerveControllerCommand(
            RightNote,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,
            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);
        return RightNoteCommand;
    }
    if (selected == "to middle note") {
        Trajectory MiddleNote =
        TrajectoryGenerator.generateTrajectory(
            m_robotDrive.getPose(),
            List.of(),
            new Pose2d(Units.inchesToMeters(noteX), Units.inchesToMeters(middleNoteY), 
                new Rotation2d(getRobotDegreeToTarget((m_robotDrive.getPose().getY() - middleNoteY), (m_robotDrive.getPose().getX() - noteX)))),
            config);

        SwerveControllerCommand MiddleNoteCommand =
        new SwerveControllerCommand(
            MiddleNote,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,
            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);
        return MiddleNoteCommand;
    }
    if (selected == "to speaker") {
        Trajectory Speaker =
        TrajectoryGenerator.generateTrajectory(
            m_robotDrive.getPose(),
            List.of(),
            new Pose2d(Units.inchesToMeters(speakerX), Units.inchesToMeters(speakerY), 
                new Rotation2d(0)),
            config);

        SwerveControllerCommand SpeakerCommand =
        new SwerveControllerCommand(
            Speaker,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,
            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);
        return SpeakerCommand;
    }
    if (selected == "to amp") {
        Trajectory Amp =
        TrajectoryGenerator.generateTrajectory(
            m_robotDrive.getPose(),
            List.of(),
            new Pose2d(Units.inchesToMeters(ampX), Units.inchesToMeters(ampY), new Rotation2d(Units.radiansToDegrees(90))),
            config);
            

        SwerveControllerCommand AmpCommand =
        new SwerveControllerCommand(
            Amp,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,
            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);
        return AmpCommand;
    }
    return nocommand;
  }

  public Command getAutonomousCommand() {

        chooserStream.forEach(
            e -> {
                e.setDefaultOption("nothing this step", "nothing this step");
                e.addOption("intake eject", "intake eject");
                e.addOption("intake shooter feed", "intake shooter feed");
                e.addOption("intake to ground (will spin intake motors)", "intake to ground");
                e.addOption("intake to stow", "intake to stow");
                e.addOption("intake to amp", "intake to amp");
                e.addOption("to left note", "to left note");
                e.addOption("to right note", "to right note");
                e.addOption("to middle note", "to middle note");
                e.addOption("to speaker", "to speaker");
                e.addOption("to amp", "to amp");
            }
        );

        return Commands.sequence(
                shootercommand,
                chosenautonomouscommand(firstChooser.getSelected()),
                chosenautonomouscommand(secondChooser.getSelected()),
                chosenautonomouscommand(thirdChooser.getSelected()),
                new InstantCommand(() -> m_robotDrive.drive(0,0,0,false,true)));
    }
}