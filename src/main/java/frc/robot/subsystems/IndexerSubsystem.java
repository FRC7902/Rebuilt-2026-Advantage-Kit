package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

/** AdvantageKit Indexer Subsystem, capable of replaying the indexer. */
public class IndexerSubsystem extends SubsystemBase {

  /**
   * AdvantageKit identifies inputs via the "Replay Bubble". Everything going to the SMC is an
   * Output. Everything coming from the SMC is an Input.
   */
  @AutoLog
  public static class IndexerInputs {

    public AngularVelocity velocity = DegreesPerSecond.of(0);
    public Voltage volts = Volts.of(0);
    public Current current = Amps.of(0);
  }

  private final IndexerInputsAutoLogged indexerInputs = new IndexerInputsAutoLogged();

  private final SparkMax motor = new SparkMax(21, MotorType.kBrushless);

  private final SmartMotorControllerConfig smcConfig =
      new SmartMotorControllerConfig(this)
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
          .withIdleMode(MotorMode.COAST)
          .withTelemetry("IndexerMotor", TelemetryVerbosity.HIGH)
          .withStatorCurrentLimit(Amps.of(40))
          .withMotorInverted(false)
          .withControlMode(ControlMode.OPEN_LOOP);
  private final SmartMotorController smc = new SparkWrapper(motor, DCMotor.getNEO(1), smcConfig);

  /** Update the AdvantageKit "inputs" (data coming from the SMC) */
  private void updateInputs() {
    indexerInputs.velocity = smc.getMechanismVelocity();
    indexerInputs.volts = smc.getVoltage();
    indexerInputs.current = smc.getStatorCurrent();
  }

  public IndexerSubsystem() {}

  /**
   * Gets the current velocity of the indexer.
   *
   * @return Indexer velocity.
   */
  public AngularVelocity getVelocity() {
    return indexerInputs.velocity;
  }

  /**
   * Set the voltage of the indexer.
   *
   * @param volts Voltage to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setVoltage(Voltage volts) {
    return this.runOnce(
            () -> {
              Logger.recordOutput("Indexer/Voltage", volts);
              smc.setVoltage(volts);
            })
        .withName("IndexerSetVoltage");
  }

  /**
   * Set the dutycycle of the indexer.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command set(double dutyCycle) {
    return this.runOnce(
            () -> {
              Logger.recordOutput("Indexer/DutyCycle", dutyCycle);
              smc.setDutyCycle(dutyCycle);
            })
        .withName("IndexerSetDutyCycle");
  }

  /**
   * DutyCycle supplier controlling the indexer
   *
   * @param dutyCycle Dutycyle supplier
   * @return Command
   */
  public Command setDutyCycle(Supplier<Double> dutyCycle) {
    return this.runOnce(
            () -> {
              double appliedDutyCycle = dutyCycle.get();
              Logger.recordOutput("Indexer/DutyCycle", appliedDutyCycle);
              smc.setDutyCycle(appliedDutyCycle);
            })
        .withName("IndexerSetDutyCycleSupplier");
  }

  @Override
  public void simulationPeriodic() {
    smc.simIterate();
  }

  @Override
  public void periodic() {
    updateInputs();
    Logger.processInputs("Indexer", indexerInputs);
    smc.updateTelemetry();
  }
}
