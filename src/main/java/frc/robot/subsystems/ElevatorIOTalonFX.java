package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

/** Single IO implementation - uses YAMS Elevator mechanism with SmartMotorController telemetry. */
public class ElevatorIOTalonFX implements ElevatorIO {

  private final Elevator elevator;
  private final SmartMotorController motor;

  SmartMotorControllerConfig smcConfig;

  public ElevatorIOTalonFX(SubsystemBase subsystem, int canId) {
    TalonFX talonFX = new TalonFX(canId);

    // Step 1: Create SmartMotorControllerConfig
    smcConfig =
        new SmartMotorControllerConfig(subsystem)
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 4)))
            .withMechanismCircumference(Inches.of(1.5 * Math.PI)) // Pulley circumference
            .withClosedLoopController(10, 0, 0.5)
            .withFeedforward(new ElevatorFeedforward(0.1, 0.2, 0.5, 0.01))
            .withTrapezoidalProfile(MetersPerSecond.of(1.0), MetersPerSecondPerSecond.of(2.0));

    // Step 2: Create SmartMotorController (TalonFXWrapper)
    SmartMotorController smc = new TalonFXWrapper(talonFX, DCMotor.getKrakenX60(1), smcConfig);

    // Step 3: Create ElevatorConfig with the SmartMotorController
    ElevatorConfig elevatorConfig =
        new ElevatorConfig(smc)
            .withDrumRadius(Inches.of(0.75)) // Drum radius for pulley
            .withMass(Pounds.of(10)) // Carriage mass - used for simulation physics
            .withHardLimits(Meters.of(0), Meters.of(1.5)) // Physical hard stops for sim
            .withSoftLimits(Meters.of(0.02), Meters.of(1.2))
            .withStartingHeight(Meters.of(0.5))
            .withTelemetry("Elevator", TelemetryVerbosity.HIGH);

    // Step 4: Create Elevator mechanism - handles simulation automatically!
    this.elevator = new Elevator(elevatorConfig);

    // Get reference to underlying SmartMotorController for telemetry
    this.motor = elevator.getMotor();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // Pull telemetry data from the underlying SmartMotorController
    inputs.positionMeters = motor.getMeasurementPosition().in(Meters);
    inputs.velocityMetersPerSec = motor.getMeasurementVelocity().in(MetersPerSecond);
    inputs.appliedVolts = motor.getVoltage().in(Volts);
    inputs.supplyCurrentAmps = motor.getSupplyCurrent().map(c -> c.in(Amps)).orElse(0.0);
    inputs.statorCurrentAmps = motor.getStatorCurrent().in(Amps);
    inputs.temperatureCelsius = motor.getTemperature().in(Celsius);
    inputs.targetPositionMeters =
        motor
            .getMechanismPositionSetpoint()
            .map(smcConfig::convertFromMechanism)
            .map(d -> d.in(Meters))
            .orElse(0.0);
  }

  @Override
  public void setTargetHeight(double meters) {
    // Use SmartMotorController's setPosition method with Distance
    motor.setPosition(Meters.of(meters));
  }

  @Override
  public void stop() {
    motor.setVoltage(Volts.of(0));
  }

  /** Access the Elevator mechanism for command helpers like run() and runTo() */
  public Elevator getElevator() {
    return elevator;
  }
}
