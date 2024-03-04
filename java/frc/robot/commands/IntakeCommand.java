// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.IntakeSubsystem.PivotTarget;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class IntakeCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubsystem m_intake;
  private final Joystick m_driverjoystick;
  private final XboxController m_secondarycontroller;

  /**
   * Creates a new ExampleCommand.
   *
   * @param m_intake The subsystem used by this command.
   */
  public IntakeCommand(IntakeSubsystem intake, Joystick joystick, XboxController secondarycontroller) {
    m_intake = intake;
    m_driverjoystick = joystick;
    m_secondarycontroller = secondarycontroller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake);
  }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_intake.getIntakeState() != IntakeState.INTAKE) {
      m_intake.stopIntake();
    }
    if (m_driverjoystick.getTrigger()) {
      if (m_intake.m_periodicIO.pivot_target == PivotTarget.STOW) {
        m_intake.feedShooter();
      } else {
        m_intake.eject();
      }
    }
    if (m_secondarycontroller.getAButton()) {
      m_intake.goToAmp();
    }
    if (m_secondarycontroller.getBButton()) {
      m_intake.goToGround();
    }
    if (m_secondarycontroller.getXButton()) {
      m_intake.goToStow();
    }
    if (m_secondarycontroller.getYButton()) {
      m_intake.goToSource();
    }
    if ((m_intake.m_periodicIO.pivot_target == PivotTarget.GROUND) && !m_driverjoystick.getTrigger()) {
      m_intake.m_periodicIO.intake_state = IntakeState.INTAKE;
    }
    if (m_intake.m_periodicIO.pivot_target == PivotTarget.STOW) {
      //m_intake.pulse();
    }
    if (m_secondarycontroller.getPOV() == 0) {
      m_intake.m_periodicIO.intake_state = IntakeState.SHOOT_OUT_A_LITTLE;
    }
    if (m_secondarycontroller.getPOV() == 180) {
      m_intake.m_periodicIO.intake_state = IntakeState.SHOOT_IN_A_LITTLE;
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
