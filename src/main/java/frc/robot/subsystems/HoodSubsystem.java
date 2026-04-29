package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class HoodSubsystem extends SubsystemBase {

  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  public HoodSubsystem(int canId) {
    this.io = new HoodIOTalonFX(this, canId);
  }

  public HoodSubsystem(HoodIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // Update and log inputs every cycle
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);
  }

  /** Command to move the hood to a target angle. Uses run() for continuous control. */
  public Command setAngle(double rotations) {
    return run(() -> io.setTargetAngle(rotations)).withName("Hood.setAngle(" + rotations + ")");
  }

  /**
   * Command to move the hood to a target angle and finish when reached. Uses runTo() pattern - be
   * careful with default commands!
   */
  public Command goToAngle(double rotations) {
    return run(() -> io.setTargetAngle(rotations))
        .until(() -> isNear(Rotations.of(rotations), Rotations.of(0.01)))
        .withName("Hood.goToAngle(" + rotations + ")");
  }

  /** Returns true if the hood is within tolerance of a target position using WPILib's isNear(). */
  public boolean isNear(Angle target, Angle tolerance) {
    return Rotations.of(inputs.angleRotations).isNear(target, tolerance);
  }

  /** Returns the current hood position in rotations. */
  public double getAngleRotations() {
    return inputs.angleRotations;
  }

  /** Command to stop the hood. */
  public Command stop() {
    return runOnce(() -> io.stop()).withName("Hood.stop");
  }

  public Pose3d getPose3d() {
    return new Pose3d(
        new Translation3d(0.2286, 0, 0.4572),
        new Rotation3d(
            0, (0.5 * Math.sin(2 * Timer.getTimestamp()) + 0.5) * Units.degreesToRadians(40), 0));
  }
}
