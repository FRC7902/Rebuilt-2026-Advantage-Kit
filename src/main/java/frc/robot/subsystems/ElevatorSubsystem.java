package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public ElevatorSubsystem(int canId) {
    this.io = new ElevatorIOTalonFX(this, canId);
  }

  public ElevatorSubsystem(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  /** Command to move the elevator to a target height. */
  public Command setHeight(double meters) {
    return run(() -> io.setTargetHeight(meters)).withName("Elevator.setHeight(" + meters + ")");
  }

  /**
   * Command to move the elevator to a target height and finish when reached. Adjust tolerance as
   * needed.
   */
  public Command goToHeight(double meters) {
    return run(() -> io.setTargetHeight(meters))
        .until(() -> isNear(Meters.of(meters), Meters.of(0.01)))
        .withName("Elevator.goToHeight(" + meters + ")");
  }

  /** Returns true if the elevator is within tolerance of a target position. */
  public boolean isNear(Distance target, Distance tolerance) {
    return Meters.of(inputs.positionMeters).isNear(target, tolerance);
  }

  /** Returns the current elevator position in meters. */
  public double getPositionMeters() {
    return inputs.positionMeters;
  }

  /** Command to stop the elevator. */
  public Command stop() {
    return runOnce(() -> io.stop()).withName("Elevator.stop");
  }
}
