// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ClimberCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimberSubsystem m_subsystem;
  private final XboxController m_secondarycontroller;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ClimberCommand(ClimberSubsystem subsystem, XboxController secondarycontroller) {
    m_subsystem = subsystem;
    m_secondarycontroller = secondarycontroller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean didsomethingright = false;
    boolean didsomethingleft = false;
    if (m_secondarycontroller.getRightTriggerAxis() > 0.05) {
      didsomethingright = true;
      m_subsystem.moveRightClimber(true, (Constants.DriveConstants.climberArmSpeed));
    }
    if (m_secondarycontroller.getRightBumper()) {
      didsomethingright = true;
      m_subsystem.moveRightClimber(false, (Constants.DriveConstants.climberArmSpeed));
    }

    if (m_secondarycontroller.getLeftTriggerAxis() > 0.05) {
      didsomethingleft = true;
      m_subsystem.moveLeftClimber(true, (Constants.DriveConstants.climberArmSpeed));
    }
    if (m_secondarycontroller.getLeftBumper()) {
      didsomethingleft = true;
      m_subsystem.moveLeftClimber(false, (Constants.DriveConstants.climberArmSpeed));
    }
    if (!didsomethingleft) {
      m_subsystem.setZeroLeft();
    }
    if (!didsomethingright) {
      m_subsystem.setZeroRight();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}