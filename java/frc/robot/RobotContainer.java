// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.autocommands.FiveSecondWait;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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
  XboxController xboxController = new XboxController(OIConstants.xboxPort);
  XboxController logitechController = new XboxController(OIConstants.logitechPort);

  public final DriveSubsystem m_robotDrive = new DriveSubsystem(xboxController);
  public final IntakeSubsystem m_intake = IntakeSubsystem.getInstance();
  public final ShooterSubsystem m_shooter = new ShooterSubsystem();
  public final ClimberSubsystem m_climber = new ClimberSubsystem();

  private final IntakeCommand m_intakeCommand = new IntakeCommand(m_intake, xboxController, logitechController);
  private final ClimberCommand climbercommand = new ClimberCommand(m_climber, logitechController);
  private final ShooterCommand shootercommand = new ShooterCommand(m_shooter);

  //ClimberSubsystem climbersubsystem = new ClimberSubsystem();
  //ClimberCommand climbercommand = new ClimberCommand(climbersubsystem, m_secondaryController);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private final SendableChooser<String> firstChooser = new SendableChooser<>();
  private final SendableChooser<String> secondChooser = new SendableChooser<>();
  private final SendableChooser<String> thirdChooser = new SendableChooser<>();
  private final SendableChooser<String> fourthChooser = new SendableChooser<>();
  private final SendableChooser<String> fifthChooser = new SendableChooser<>();
  private final SendableChooser<String> sixthChooser = new SendableChooser<>();
  private final SendableChooser<String> seventhChooser = new SendableChooser<>();
  private final SendableChooser<String> eighthChooser = new SendableChooser<>();
  private final SendableChooser<String> ninthChooser = new SendableChooser<>();
  private final SendableChooser<String> tenthChooser = new SendableChooser<>();
  private final SendableChooser<String> eleventhChooser = new SendableChooser<>();
  private final SendableChooser<String> twelfthChooser = new SendableChooser<>();
  private final SendableChooser<String> thirteenthChooser = new SendableChooser<>();
  private final SendableChooser<String> fourtheenthChooser = new SendableChooser<>();
  private final SendableChooser<String> fifteenthChooser = new SendableChooser<>();
  private final SendableChooser<String> sixteenthChooser = new SendableChooser<>();
  private final SendableChooser<String> seventeethChooser = new SendableChooser<>();
  private final SendableChooser<String> eighteenthChooser = new SendableChooser<>();
  private final SendableChooser<String> ninteenthChooser = new SendableChooser<>();
  private final SendableChooser<String> twentiethChooser = new SendableChooser<>();
  private final SendableChooser<String> twentyfirstChooser = new SendableChooser<>();
  private final SendableChooser<String> twentysecondChooser = new SendableChooser<>();
  private final SendableChooser<String> twentythirdChooser = new SendableChooser<>();
  private final SendableChooser<String> twentyfourthChooser = new SendableChooser<>();

  private final SendableChooser<String> presetAuto = new SendableChooser<>();
 
  @SuppressWarnings("unchecked")
  private final SendableChooser<String>[] chooserArray = new SendableChooser[24];  
  private final Stream<SendableChooser<String>> chooserStream;


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

  //keep in mind for the amp the robot also has to turn +/-90 degrees
  //the angle for the rest of the paths should theoretically be calculatable using trigonoemtry but i need to do that, right now they just face 0 when going (WHICH WILL NOT WORK) (probably)
  private double ampY = 323-((Constants.DriveConstants.kWheelBase)/2 + bumperthickness);
  private double ampX = 72.5;
  private double ampAngle = -90; //this will be set to -90 if the robot is on team red

  //dumping the red team X values here so i can add the logic later
  //the length of the field in inches is 653.2 (X)
  private final double rednoteX = 539.2;
  private final double redspeakerX = 653.2-speakerX;
  private final double redampX = 653.2-76.1;
  
  public static double getRobotDegreeToTarget(double deltaY, double deltaX) {
    if (deltaY >= 0) {
        return 180-Math.abs(Math.toDegrees(Math.atan(deltaY/deltaX)));
    } else {
        return -180-Math.abs(Math.toDegrees(Math.atan(deltaY/deltaX)));
    }
  }

  public RobotContainer() {

    if (DriverStation.getAlliance().get() == Alliance.Red) {
        //noteX = rednoteX;
        //speakerX = redspeakerX;
        //ampX = redampX;
        //ampAngle = 90;
    }


    //sendable chooser
    chooserArray[0] = firstChooser;
    chooserArray[1] = secondChooser;
    chooserArray[2] = thirdChooser;
    chooserArray[3] = fourthChooser;
    chooserArray[4] = fifthChooser;
    chooserArray[5] = sixthChooser;
    chooserArray[6] = seventhChooser;
    chooserArray[7] = eighthChooser;
    chooserArray[8] = ninthChooser;
    chooserArray[9] = tenthChooser;
    chooserArray[10] = eleventhChooser;
    chooserArray[11] = twelfthChooser;
    chooserArray[12] = thirteenthChooser;
    chooserArray[13] = fourtheenthChooser;
    chooserArray[14] = fifteenthChooser;
    chooserArray[15] = sixteenthChooser;
    chooserArray[16] = seventeethChooser;
    chooserArray[17] = eighteenthChooser;
    chooserArray[18] = ninteenthChooser;
    chooserArray[19] = twentiethChooser;
    chooserArray[20] = twentyfirstChooser;
    chooserArray[21] = twentysecondChooser;
    chooserArray[22] = twentythirdChooser;
    chooserArray[23] = twentyfourthChooser;


    System.out.println(chooserArray);

    chooserStream = Arrays.stream(chooserArray);

    firstChooser.setDefaultOption("===STEP 1===", "first chooser");
    secondChooser.setDefaultOption("===STEP 2===", "second chooser");
    thirdChooser.setDefaultOption("===STEP 3===", "third chooser");
    fourthChooser.setDefaultOption("===STEP 4===", "fourth chooser");
    fifthChooser.setDefaultOption("===STEP 5===", "fifth chooser");
    sixthChooser.setDefaultOption("===STEP 6===", "sixth chooser");
    seventhChooser.setDefaultOption("===STEP 7===", "seventh chooser");
    eighthChooser.setDefaultOption("===STEP 8===", "eighth chooser");
    ninthChooser.setDefaultOption("===STEP 9===", "ninth chooser");
    tenthChooser.setDefaultOption("===STEP 10===", "tenth chooser");
    eleventhChooser.setDefaultOption("===STEP 11===", "eleventh chooser");
    twelfthChooser.setDefaultOption("===STEP 12===", "twelfth chooser");    
    thirteenthChooser.setDefaultOption("===STEP 13===", "thirteenth chooser");
    fourtheenthChooser.setDefaultOption("===STEP 14===", "fourteenth chooser");  
    fifteenthChooser.setDefaultOption("===STEP 15===", "fifteenth chooser");
    sixteenthChooser.setDefaultOption("===STEP 16===", "sixteenth chooser");
    seventeethChooser.setDefaultOption("===STEP 17===", "seventeeth chooser");    
    eighteenthChooser.setDefaultOption("===STEP 18===", "eighteenth chooser");
    ninteenthChooser.setDefaultOption("===STEP 19===", "ninteenth chooser");    
    twentiethChooser.setDefaultOption("===STEP 20===", "twentieth chooser");
    twentyfirstChooser.setDefaultOption("===STEP 21===", "twentyfirst chooser");
    twentysecondChooser.setDefaultOption("===STEP 22===", "twentysecond chooser");
    twentythirdChooser.setDefaultOption("===STEP 23===", "twentythird chooser");
    twentyfourthChooser.setDefaultOption("===STEP 24===", "twentyfourth chooser");    

    presetAuto.setDefaultOption("===PRESET AUTOS===", "nothing");
    presetAuto.addOption("Four Note, Speaker, From Top", "SPEAKER_4FromTop");

    chooserStream.forEach(
        e -> {
            e.addOption("5_sec_wait", "5_sec_wait");
            e.addOption("i_eject", "intake eject");
            e.addOption("i_feed", "intake shooter feed");
            e.addOption("i_ground", "intake to ground");
            e.addOption("i_stow", "intake to stow");
            e.addOption("i_amp", "intake to amp");
            e.addOption("r_topnote", "to top note");
            e.addOption("r_bottomnote", "to bottom note");
            e.addOption("r_middlenote", "to middle note");
            e.addOption("r_speaker", "to speaker");
            e.addOption("r_amp", "to amp");
        }
    );

    SmartDashboard.putData("first autonomous", firstChooser);
    SmartDashboard.putData("second autonomous", secondChooser);
    SmartDashboard.putData("third autonomous", thirdChooser);
    SmartDashboard.putData("fourth autonomous", fourthChooser);
    SmartDashboard.putData("fifth autonomous", fifthChooser);
    SmartDashboard.putData("sixth autonomous", sixthChooser);
    SmartDashboard.putData("seventh autonomous", seventhChooser);
    SmartDashboard.putData("eighth autonomous", eighthChooser);
    SmartDashboard.putData("ninth autonomous", ninthChooser);
    SmartDashboard.putData("tenth autonomous", tenthChooser);
    SmartDashboard.putData("eleventh autonomous", eleventhChooser);
    SmartDashboard.putData("twelfth autonomous", twelfthChooser);
    SmartDashboard.putData("thirteenth autonomous", thirteenthChooser);
    SmartDashboard.putData("fourteenth autonomous", fourtheenthChooser);
    SmartDashboard.putData("fifteenth autonomous", fifteenthChooser);
    SmartDashboard.putData("sixteenth autonomous", sixteenthChooser);
    SmartDashboard.putData("seventeeth autonomous", seventeethChooser);
    SmartDashboard.putData("eighteenth autonomous", eighteenthChooser);
    SmartDashboard.putData("ninteenth autonomous", ninteenthChooser);
    SmartDashboard.putData("twentieth autonomous", twentiethChooser);
    SmartDashboard.putData("twentyfirst autonomous", twentyfirstChooser);
    SmartDashboard.putData("twentysecond autonomous", twentysecondChooser);
    SmartDashboard.putData("twentythird autonomous", twentythirdChooser);
    SmartDashboard.putData("twentyfourth autonomous", twentyfourthChooser);

    SmartDashboard.putData("preset auto", presetAuto);

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
                -MathUtil.applyDeadband(Math.pow(xboxController.getLeftY(), 3), Math.pow(OIConstants.kDriveDeadband, 3)),
                -MathUtil.applyDeadband(Math.pow(xboxController.getLeftX(), 3), Math.pow(OIConstants.kDriveDeadband, 3)),
                -MathUtil.applyDeadband(Math.pow(xboxController.getRightX(), 1), Math.pow(OIConstants.kDriveDeadband, 1)),
                false, true),
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

    if (selected == "5_sec_wait") {
        return new FiveSecondWait();
    }
    if (selected == "intake eject") {
        return new IntakeEject(m_intake);
    }
    if (selected == "intake shooter feed") {
        return new IntakeShooter(m_intake);
    }
    if (selected == "intake to ground") {
        return new IntakeGround(m_intake);
    }
    if (selected == "intake to stow") {
        return new IntakeStow(m_intake);
    }
    if (selected == "intake to amp") {
        return new IntakeAmp(m_intake);
    }
    if (selected == "to top note") {
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
    if (selected == "to bottom note") {
        Trajectory RightNote =
        TrajectoryGenerator.generateTrajectory(
            m_robotDrive.getPose(),
            List.of(),
            new Pose2d(Units.inchesToMeters(noteX), Units.inchesToMeters(rightNoteY), 
                new Rotation2d(Units.degreesToRadians(180))),
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
                new Rotation2d(Units.degreesToRadians(180))),
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
                new Rotation2d(Units.degreesToRadians(180))),
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
            new Pose2d(Units.inchesToMeters(ampX), Units.inchesToMeters(ampY), new Rotation2d(Units.degreesToRadians(ampAngle))),
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
    if (selected == "test") {
        Trajectory LeftNote =
        TrajectoryGenerator.generateTrajectory(
            m_robotDrive.getPose(),
            List.of(),
            new Pose2d(-0.25, 0, 
                //new Rotation2d(getRobotDegreeToTarget((m_robotDrive.getPose().getY() - 0.25), (m_robotDrive.getPose().getX() - -0.5)))),
                new Rotation2d(0)),
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
    return new NoCommand();
  }



  public Command getAutonomousCommand() {
        if (presetAuto.getSelected() != "nothing") {
            if (presetAuto.getSelected() == "SPEAKER_4FromTop") {
                return Commands.sequence(
                    new ShooterCommand(m_shooter),
                    new IntakeShooter(m_intake),
                    chosenautonomouscommand("to speaker"),
                    new IntakeGround(m_intake),
                    chosenautonomouscommand("to top note"),
                    new IntakeStow(m_intake),
                    chosenautonomouscommand("to speaker"),
                    new IntakeShooter(m_intake),
                    new IntakeGround(m_intake),
                    chosenautonomouscommand("to middle note"),
                    new IntakeStow(m_intake),
                    chosenautonomouscommand("to speaker"),
                    new IntakeShooter(m_intake),
                    new IntakeGround(m_intake),
                    chosenautonomouscommand("to bottom note"),
                    new IntakeStow(m_intake),
                    chosenautonomouscommand("to speaker"),
                    new IntakeEject(m_intake),
                    chosenautonomouscommand("to middle note"),
                    new InstantCommand(() -> m_robotDrive.drive(0,0,0,false,true)));
            }
        } 
        else {
            return Commands.sequence(
                    //new ShooterCommand(m_shooter),
                    chosenautonomouscommand(firstChooser.getSelected()),
                    chosenautonomouscommand(secondChooser.getSelected()),
                    chosenautonomouscommand(thirdChooser.getSelected()),
                    chosenautonomouscommand(fourthChooser.getSelected()),
                    chosenautonomouscommand(fifthChooser.getSelected()),
                    chosenautonomouscommand(sixthChooser.getSelected()),
                    chosenautonomouscommand(seventhChooser.getSelected()),
                    chosenautonomouscommand(eighthChooser.getSelected()),
                    chosenautonomouscommand(ninthChooser.getSelected()),
                    chosenautonomouscommand(tenthChooser.getSelected()),
                    chosenautonomouscommand(eleventhChooser.getSelected()),
                    chosenautonomouscommand(twelfthChooser.getSelected()),
                    chosenautonomouscommand(thirteenthChooser.getSelected()),
                    chosenautonomouscommand(fourtheenthChooser.getSelected()),
                    chosenautonomouscommand(fifteenthChooser.getSelected()),
                    chosenautonomouscommand(sixteenthChooser.getSelected()),
                    chosenautonomouscommand(seventeethChooser.getSelected()),
                    chosenautonomouscommand(eighteenthChooser.getSelected()),
                    chosenautonomouscommand(ninteenthChooser.getSelected()),
                    chosenautonomouscommand(twentiethChooser.getSelected()),
                    chosenautonomouscommand(twentyfirstChooser.getSelected()),
                    chosenautonomouscommand(twentysecondChooser.getSelected()),
                    chosenautonomouscommand(twentythirdChooser.getSelected()),
                    chosenautonomouscommand(twentyfourthChooser.getSelected()),
                    new InstantCommand(() -> m_robotDrive.drive(0,0,0,false,true)));
        }
        return null;
    }
}