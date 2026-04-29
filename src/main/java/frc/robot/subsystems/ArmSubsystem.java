package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase {

  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  public ArmSubsystem(int canId) {
    this.io = new ArmIOTalonFX(this, canId);
  }

  public ArmSubsystem(ArmIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // Update and log inputs every cycle
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
  }

  /** Command to move the arm to a target angle. Uses run() for continuous control. */
  public Command setAngle(double rotations) {
    return run(() -> io.setTargetAngle(rotations)).withName("Arm.setAngle(" + rotations + ")");
  }

  /**
   * Command to move the arm to a target angle and finish when reached. Uses runTo() pattern - be
   * careful with default commands!
   */
  public Command goToAngle(double rotations) {
    return run(() -> io.setTargetAngle(rotations))
        .until(() -> isNear(Rotations.of(rotations), Rotations.of(0.01)))
        .withName("Arm.goToAngle(" + rotations + ")");
  }

  /** Returns true if the arm is within tolerance of a target position using WPILib's isNear(). */
  public boolean isNear(Angle target, Angle tolerance) {
    return Rotations.of(inputs.positionRotations).isNear(target, tolerance);
  }

  /** Returns the current arm position in rotations. */
  public double getPositionRotations() {
    return inputs.positionRotations;
  }

  /** Command to stop the arm. */
  public Command stop() {
    return runOnce(() -> io.stop()).withName("Arm.stop");
  }
}
