package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class LinearIntakeSubsystem extends SubsystemBase {

  private final LinearIntakeIO io;
  private final LinearIntakeIOInputsAutoLogged inputs = new LinearIntakeIOInputsAutoLogged();

  public LinearIntakeSubsystem(int canId) {
    this.io = new LinearIntakeIOTalonFX(this, canId);
  }

  public LinearIntakeSubsystem(LinearIntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("LinearIntake", inputs);
  }

  /** Command to move the linear intake to a target extension. */
  public Command setExtension(double meters) {
    return run(() -> io.setTargetExtension(meters))
        .withName("LinearIntake.setExtension(" + meters + ")");
  }

  /**
   * Command to move the linear intake to a target extension and finish when reached. Adjust
   * tolerance as needed.
   */
  public Command goToExtension(double meters) {
    return run(() -> io.setTargetExtension(meters))
        .until(() -> isNear(Meters.of(meters), Meters.of(0.01)))
        .withName("LinearIntake.goToExtension(" + meters + ")");
  }

  /** Returns true if the linear intake is within tolerance of a target position. */
  public boolean isNear(Distance target, Distance tolerance) {
    return Meters.of(inputs.extensionMeters).isNear(target, tolerance);
  }

  /** Returns the current linear intake extension in meters. */
  public double getExtensionMeters() {
    return inputs.extensionMeters;
  }

  /** Command to stop the linear intake. */
  public Command stop() {
    return runOnce(() -> io.stop()).withName("LinearIntake.stop");
  }

  public Pose3d getPose3d() {
    double angle = Math.toRadians(180 + 24.159);

    // Convert distance vector to x and z components based on angle
    double x = inputs.extensionMeters * Math.cos(angle);
    double z = inputs.extensionMeters * Math.sin(angle);
    return new Pose3d(new Translation3d(x, 0, z), new Rotation3d());
  }
}
