package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  public ClimberSubsystem() {}

  public CANSparkMax leftclimbermotor = new CANSparkMax(9, MotorType.kBrushless);
  public CANSparkMax rightclimbermotor = new CANSparkMax(10, MotorType.kBrushless);

  public void moveLeftClimber(boolean isGoingUp, double speed) {
    if (isGoingUp) {
      leftclimbermotor.set(speed);
    } else {leftclimbermotor.set(-speed);}
  }
 
    public void moveRightClimber(boolean isGoingUp, double speed) {
    if (isGoingUp) {
      rightclimbermotor.set(speed);
    } else {rightclimbermotor.set(-speed);}
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}