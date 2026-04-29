package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public interface LinearIntakeIO {

  @AutoLog
  public static class LinearIntakeIOInputs {
    public double extensionMeters = 0.0;
    public double extensionMetersPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
    public double temperatureCelsius = 0.0;
    public double targetExtensionMeters = 0.0;
  }

  default void updateInputs(LinearIntakeIOInputs inputs) {}

  default void setTargetExtension(double meters) {}

  default void stop() {}
}
