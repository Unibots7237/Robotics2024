package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  public ShooterSubsystem() {}

  public CANSparkMax leftshooter = new CANSparkMax(Constants.kShooterLeftMotorId, MotorType.kBrushless);
  public CANSparkMax rightshooter = new CANSparkMax(Constants.kShooterRightMotorId, MotorType.kBrushless);

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public void ShooterSubsystem () {
    //leftshooter.setInverted(true);
    leftshooter.setIdleMode(IdleMode.kCoast);
    rightshooter.setIdleMode(IdleMode.kCoast);
  }
  
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  public void shoot() {
    leftshooter.set(1);
    rightshooter.set(-1);
  }

  public void stop() {
    leftshooter.set(0);
    rightshooter.set(0);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}