package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Helpers;

public class IntakeSubsystem extends Subsystem {
  private static final double k_pivotMotorP = 0.09;
  private static final double k_pivotMotorI = 0.0;
  private static final double k_pivotMotorD = 0.0;

  private final PIDController m_pivotPID = new PIDController(k_pivotMotorP, k_pivotMotorI, k_pivotMotorD);

  //private static final double kS = 0.0;
  //private static final double kG = -1;
  //private static final double kV = 0.0;
  //private static final double kA = 0.0;
  //private final ArmFeedforward feedforward = new ArmFeedforward(kS, kG, kV, kA);

  private CANSparkMax mIntakeMotor;
  private CANSparkMax mPivotMotor;

  public final AbsoluteEncoder m_pivotEncoder;
  private final DigitalInput m_IntakeLimitSwitch = new DigitalInput(Constants.Intake.k_intakeLimitSwitchId);


  /*-------------------------------- Private instance variables ---------------------------------*/
  private static IntakeSubsystem mInstance;
  public PeriodicIO m_periodicIO;

  public static IntakeSubsystem getInstance() {
    if (mInstance == null) {
      mInstance = new IntakeSubsystem();
    }
    return mInstance;
  }

  public IntakeSubsystem() {
    super("IntakeSubystem");

    mIntakeMotor = new CANSparkMax(Constants.Intake.kIntakeMotorId, MotorType.kBrushless);
    mIntakeMotor.restoreFactoryDefaults();
    mIntakeMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

    mPivotMotor = new CANSparkMax(Constants.Intake.kPivotMotorId, MotorType.kBrushless);
    mPivotMotor.restoreFactoryDefaults();
    m_pivotEncoder = mPivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
    mPivotMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    mPivotMotor.setSmartCurrentLimit(10);

    m_periodicIO = new PeriodicIO();
  }

  public static class PeriodicIO {
    // Input: Desired state
    public PivotTarget pivot_target = PivotTarget.STOW;
    public IntakeState intake_state = IntakeState.NONE;

    // Output: Motor set values
    double intake_pivot_voltage = 0.0;
    double intake_speed = 0.0;
  }

  public enum PivotTarget {
    NONE,
    GROUND,
    SOURCE,
    AMP,
    STOW
  }

  public enum IntakeState {
    NONE,
    INTAKE,
    EJECT,
    PULSE,
    FEED_SHOOTER,
    SHOOT_OUT_A_LITTLE,
    SHOOT_IN_A_LITTLE
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  @Override
  public void periodic() {
    checkAutoTasks();

    // Pivot control
    double pivot_angle = pivotTargetToAngle(m_periodicIO.pivot_target);
    m_periodicIO.intake_pivot_voltage = m_pivotPID.calculate(getPivotAngleDegrees(), pivot_angle);
    // If the pivot is at exactly 0.0, it's probably not connected, so disable it
    if (m_pivotEncoder.getPosition() == 0.0) {
      m_periodicIO.intake_pivot_voltage = 0.0;
    }

    // Intake control
    m_periodicIO.intake_speed = intakeStateToSpeed(m_periodicIO.intake_state);
    putString("State", m_periodicIO.intake_state.toString());
    SmartDashboard.putNumber("Pivot Angle", getPivotAngleDegrees());
    SmartDashboard.putNumber("Intake Encoder Real Angle", m_pivotEncoder.getPosition());
  }

  @Override
  public void writePeriodicOutputs() {
    mPivotMotor.setVoltage(m_periodicIO.intake_pivot_voltage);
    //mPivotMotor.set(0.5);
    mIntakeMotor.set(m_periodicIO.intake_speed);
  }

  @Override
  public void stop() {
    m_periodicIO.intake_pivot_voltage = 0.0;
    m_periodicIO.intake_speed = 0.0;
  }

  @Override
  public void outputTelemetry() {
    putNumber("Speed", intakeStateToSpeed(m_periodicIO.intake_state));
    putNumber("Pivot/Abs Enc (get)", m_pivotEncoder.getPosition());
    putNumber("Pivot/Abs Enc (getAbsolutePosition)", m_pivotEncoder.getPosition());
    putNumber("Pivot/Abs Enc (getPivotAngleDegrees)", getPivotAngleDegrees());
    putNumber("Pivot/Setpoint", pivotTargetToAngle(m_periodicIO.pivot_target));

    putNumber("Pivot/Power", m_periodicIO.intake_pivot_voltage);
    putNumber("Pivot/Current", mPivotMotor.getOutputCurrent());

    putBoolean("Limit Switch", getIntakeHasNote());
  }

  @Override
  public void reset() {
  }

  public double pivotTargetToAngle(PivotTarget target) {
    switch (target) {
      case GROUND:
        return Constants.Intake.k_pivotAngleGround;
      case SOURCE:
        return Constants.Intake.k_pivotAngleSource;
      case AMP:
        return Constants.Intake.k_pivotAngleAmp;
      case STOW:
        return Constants.Intake.k_pivotAngleStow;
      default:
        // "Safe" default
        return 180;
    }
  }

  public double intakeStateToSpeed(IntakeState state) {
    switch (state) {
      case INTAKE:
        return Constants.Intake.k_intakeSpeed;
      case EJECT:
        return Constants.Intake.k_ejectSpeed;
      case PULSE:
        // Use the timer to pulse the intake on for a 1/16 second,
        // then off for a 15/16 second
        //if (Timer.getFPGATimestamp() % 1.0 < (1.0 / 45.0)) {
          //return Constants.Intake.k_intakeSpeed;
        //}
        //return 0.25;
      case FEED_SHOOTER:
        return Constants.Intake.k_feedShooterSpeed;
      case SHOOT_OUT_A_LITTLE:
        return -0.1;
      case SHOOT_IN_A_LITTLE:
        return 0.1;
      default:
        // "Safe" default
        return 0.0;
    }
  }

  /*---------------------------------- Custom Public Functions ----------------------------------*/

  public IntakeState getIntakeState() {
    return m_periodicIO.intake_state;
  }

  public double getPivotAngleDegrees() {
    double value = m_pivotEncoder.getPosition();

    return Units.rotationsToDegrees(Helpers.modRotations(value));
  }

  public boolean getIntakeHasNote() {
    // NOTE: this is intentionally inverted, because the limit switch is normally
    // closed
    return !m_IntakeLimitSwitch.get();
  }

  // Pivot helper functions
  public void goToGround() {
    m_periodicIO.pivot_target = PivotTarget.GROUND;
    m_periodicIO.intake_state = IntakeState.INTAKE;
  }

  public void goToSource() {
    m_periodicIO.pivot_target = PivotTarget.SOURCE;
    m_periodicIO.intake_state = IntakeState.NONE;
  }

  public void goToAmp() {
    m_periodicIO.pivot_target = PivotTarget.SOURCE;
    m_periodicIO.intake_state = IntakeState.NONE;
  }

  public void goToStow() {
    m_periodicIO.pivot_target = PivotTarget.STOW;
    m_periodicIO.intake_state = IntakeState.NONE;
  }

  // Intake helper functions
  public void intake() {
    m_periodicIO.intake_state = IntakeState.INTAKE;
  }

  public void eject() {
    m_periodicIO.intake_state = IntakeState.EJECT;
  }

  public void pulse() {
    m_periodicIO.intake_state = IntakeState.PULSE;
  }

  public void feedShooter() {
    m_periodicIO.intake_state = IntakeState.FEED_SHOOTER;
  }

  public void stopIntake() {
    m_periodicIO.intake_state = IntakeState.NONE;
    m_periodicIO.intake_speed = 0.0;
  }

  public void setState(IntakeState state) {
    m_periodicIO.intake_state = state;
  }

  public void setPivotTarget(PivotTarget target) {
    m_periodicIO.pivot_target = target;
  }

  /*---------------------------------- Custom Private Functions ---------------------------------*/
  private void checkAutoTasks() {
    // If the intake is set to GROUND, and the intake has a note, and the pivot is
    // close to it's target
    // Stop the intake and go to the SOURCE position
    if (m_periodicIO.pivot_target == PivotTarget.GROUND && getIntakeHasNote() && isPivotAtTarget()) {
      m_periodicIO.pivot_target = PivotTarget.STOW;
      m_periodicIO.intake_state = IntakeState.NONE;
    }
  }

  private boolean isPivotAtTarget() {
    return Math.abs(getPivotAngleDegrees() - pivotTargetToAngle(m_periodicIO.pivot_target)) < 5;
  }
}