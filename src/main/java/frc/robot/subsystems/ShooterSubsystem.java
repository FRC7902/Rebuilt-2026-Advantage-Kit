package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public ShooterSubsystem(int canId) {
    this.io = new ShooterIOTalonFX(this, canId);
  }

  public ShooterSubsystem(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  /** Command to set the shooter target velocity. */
  public Command setVelocity(double rotationsPerSec) {
    return run(() -> io.setTargetVelocity(rotationsPerSec))
        .withName("Shooter.setVelocity(" + rotationsPerSec + ")");
  }

  /** Command to set the shooter velocity and finish when reached. Adjust tolerance as needed. */
  public Command goToVelocity(double rotationsPerSec) {
    return run(() -> io.setTargetVelocity(rotationsPerSec))
        .until(() -> isNear(RotationsPerSecond.of(rotationsPerSec), RotationsPerSecond.of(1.0)))
        .withName("Shooter.goToVelocity(" + rotationsPerSec + ")");
  }

  /** Returns true if the shooter is within tolerance of a target velocity. */
  public boolean isNear(AngularVelocity target, AngularVelocity tolerance) {
    return RotationsPerSecond.of(inputs.velocityRotationsPerSec).isNear(target, tolerance);
  }

  /** Returns the current shooter velocity in rotations per second. */
  public double getVelocityRotationsPerSec() {
    return inputs.velocityRotationsPerSec;
  }

  /** Command to stop the shooter. */
  public Command stop() {
    return runOnce(() -> io.stop()).withName("Shooter.stop");
  }
}
