package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

/** Single IO implementation - uses YAMS FlyWheel mechanism with SmartMotorController telemetry. */
public class ShooterIOTalonFX implements ShooterIO {

  private final FlyWheel flywheel;
  private final SmartMotorController motor;

  public ShooterIOTalonFX(SubsystemBase subsystem, int canId) {
    TalonFX talonFX = new TalonFX(canId);

    // Step 1: Create SmartMotorControllerConfig
    SmartMotorControllerConfig smcConfig =
        new SmartMotorControllerConfig(subsystem)
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(1))) // Direct drive
            .withClosedLoopController(0.5, 0, 0)
            .withFeedforward(new SimpleMotorFeedforward(0.1, 0.12, 0.01));

    // Step 2: Create SmartMotorController (TalonFXWrapper)
    SmartMotorController smc = new TalonFXWrapper(talonFX, DCMotor.getKrakenX60(1), smcConfig);

    // Step 3: Create FlyWheelConfig with the SmartMotorController
    FlyWheelConfig flywheelConfig =
        new FlyWheelConfig(smc)
            .withDiameter(Inches.of(4)) // Flywheel diameter
            .withMass(Pounds.of(0.5)) // Flywheel mass - used for simulation physics
            .withSoftLimit(RPM.of(0), RPM.of(6000)) // Velocity soft limits
            .withTelemetry("Shooter", TelemetryVerbosity.HIGH);

    // Step 4: Create FlyWheel mechanism - handles simulation automatically!
    this.flywheel = new FlyWheel(flywheelConfig);

    // Get reference to underlying SmartMotorController for telemetry
    this.motor = flywheel.getMotor();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Pull telemetry data from the underlying SmartMotorController
    inputs.velocityRotationsPerSec = motor.getMechanismVelocity().in(RotationsPerSecond);
    inputs.appliedVolts = motor.getVoltage().in(Volts);
    inputs.supplyCurrentAmps = motor.getSupplyCurrent().map(c -> c.in(Amps)).orElse(0.0);
    inputs.statorCurrentAmps = motor.getStatorCurrent().in(Amps);
    inputs.temperatureCelsius = motor.getTemperature().in(Celsius);
    inputs.targetVelocityRotationsPerSec =
        motor.getMechanismSetpointVelocity().map(v -> v.in(RotationsPerSecond)).orElse(0.0);
  }

  @Override
  public void setTargetVelocity(double rotationsPerSec) {
    // Use SmartMotorController's setVelocity method
    motor.setVelocity(RotationsPerSecond.of(rotationsPerSec));
  }

  @Override
  public void stop() {
    motor.setVoltage(Volts.of(0));
  }

  /** Access the FlyWheel mechanism for command helpers like run() and runTo() */
  public FlyWheel getFlyWheel() {
    return flywheel;
  }
}
