package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {

  /**
   * Inputs that will be logged and replayed. The @AutoLog annotation generates
   * HoodIOInputsAutoLogged class.
   */
  @AutoLog
  public static class HoodIOInputs {
    public double angleRotations = 0.0;
    public double velocityRotationsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
    public double temperatureCelsius = 0.0;
    public double targetAngleRotations = 0.0;
  }

  /** Update the inputs from hardware. Called every loop cycle. */
  default void updateInputs(HoodIOInputs inputs) {}

  /** Set the target angle for the hood. */
  default void setTargetAngle(double rotations) {}

  /** Stop the hood motor. */
  default void stop() {}
}
