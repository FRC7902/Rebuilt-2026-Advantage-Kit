package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

/**
 * Single IO implementation for the arm - works in BOTH real and simulation! Uses YAMS Arm mechanism
 * class while pulling telemetry from the underlying SmartMotorController.
 */
public class ArmIOTalonFX implements ArmIO {

  private final Arm arm;
  private final SmartMotorController motor;

  public ArmIOTalonFX(SubsystemBase subsystem, int canId) {
    TalonFX talonFX = new TalonFX(canId);

    // Step 1: Create SmartMotorControllerConfig
    SmartMotorControllerConfig smcConfig =
        new SmartMotorControllerConfig(subsystem)
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 4, 3)))
            .withClosedLoopController(5, 0, 0.1)
            .withFeedforward(new ArmFeedforward(0.1, 0.3, 0.5, 0.01))
            .withTrapezoidalProfile(
                RotationsPerSecond.of(1.0), RotationsPerSecondPerSecond.of(2.0));

    // Step 2: Create SmartMotorController (TalonFXWrapper)
    SmartMotorController smc = new TalonFXWrapper(talonFX, DCMotor.getKrakenX60(1), smcConfig);

    // Step 3: Create ArmConfig with the SmartMotorController
    ArmConfig armConfig =
        new ArmConfig(smc)
            .withLength(Inches.of(18)) // Arm length - used for simulation physics
            .withMass(Pounds.of(5)) // Arm mass - used for simulation physics
            .withHardLimit(Rotations.of(-0.3), Rotations.of(0.3)) // Physical hard stops for sim
            .withSoftLimits(Rotations.of(-0.25), Rotations.of(0.25))
            .withStartingPosition(Rotations.of(0))
            .withTelemetry("Arm", TelemetryVerbosity.HIGH);

    // Step 4: Create Arm mechanism - handles simulation automatically!
    this.arm = new Arm(armConfig);

    // Get reference to underlying SmartMotorController for telemetry
    this.motor = arm.getMotor();
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    // Pull telemetry data from the underlying SmartMotorController
    inputs.positionRotations = motor.getMechanismPosition().in(Rotations);
    inputs.velocityRotationsPerSec = motor.getMechanismVelocity().in(RotationsPerSecond);
    inputs.appliedVolts = motor.getVoltage().in(Volts);
    inputs.supplyCurrentAmps = motor.getSupplyCurrent().map(c -> c.in(Amps)).orElse(0.0);
    inputs.statorCurrentAmps = motor.getStatorCurrent().in(Amps);
    inputs.temperatureCelsius = motor.getTemperature().in(Celsius);
    inputs.targetPositionRotations =
        motor.getMechanismPositionSetpoint().map(a -> a.in(Rotations)).orElse(0.0);
  }

  @Override
  public void setTargetAngle(double rotations) {
    // Use SmartMotorController's setPosition method
    motor.setPosition(Rotations.of(rotations));
  }

  @Override
  public void stop() {
    motor.setVoltage(Volts.of(0));
  }

  /** Access the Arm mechanism for command helpers like run() and runTo() */
  public Arm getArm() {
    return arm;
  }
}
