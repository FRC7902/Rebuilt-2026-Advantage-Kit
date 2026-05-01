package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.math.ExponentialProfilePIDController;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

/** AdvantageKit Elevator Subsystem, capable of replaying the elevator. */
public class LinearIntakeSubsystem extends SubsystemBase {

  /**
   * AdvantageKit identifies inputs via the "Replay Bubble". Everything going to
   * the SMC is an
   * Output. Everything coming from the SMC is an Input.
   */
  @AutoLog
  public static class LinearIntakeInputs {

    public Distance position = Meters.of(0);
    public LinearVelocity velocity = MetersPerSecond.of(0);
    public Distance setpoint = Meters.of(0);
    public Voltage volts = Volts.of(0);
    public Current current = Amps.of(0);
  }

  private final LinearIntakeInputsAutoLogged linearIntakeInputs = new LinearIntakeInputsAutoLogged();

  private final Distance chainPitch = Inches.of(0.25);
  private final int toothCount = 22;
  private final Distance circumference = chainPitch.times(toothCount);
  private final Distance radius = circumference.div(2 * Math.PI);
  private final Mass weight = Pounds.of(16);
  private final DCMotor motors = DCMotor.getNEO(1);
  private final MechanismGearing gearing = new MechanismGearing(GearBox.fromReductionStages(3, 4));
  private final SparkMax linearIntakeMotor = new SparkMax(30, SparkLowLevel.MotorType.kBrushless);
  private final SparkMax linearIntakeMotor2 = new SparkMax(31, SparkLowLevel.MotorType.kBrushless);

  private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withMechanismCircumference(circumference)
      .withClosedLoopController(
          new ExponentialProfilePIDController(
              30,
              0,
              0,
              ExponentialProfilePIDController.createElevatorConstraints(
                  Volts.of(12), motors, weight, radius, gearing)))
      .withFeedforward(new ElevatorFeedforward(0, 0.1, 0, 0))
      .withStatorCurrentLimit(Amps.of(40))
      .withMotorInverted(false)
      .withSoftLimit(Meters.of(0), Meters.of(2))
      .withGearing(gearing)
      .withIdleMode(MotorMode.BRAKE)
      .withTelemetry("LinearIntakeMotor", TelemetryVerbosity.HIGH)
      // LinearIntake motor2 follows LinearIntake motor with an inversed output.
      .withFollowers(Pair.of(linearIntakeMotor2, true));

  private final SmartMotorController motor = new SparkWrapper(linearIntakeMotor, motors, motorConfig);
  private ElevatorConfig m_config = new ElevatorConfig(motor)
      .withStartingHeight(Meters.of(0.3132))
      .withHardLimits(Meters.of(0), Meters.of(0.3132))
      .withTelemetry("LinearIntake", TelemetryVerbosity.HIGH)
      .withAngle(Degrees.of(180 + 24.159))
      .withMass(weight);
  private final Elevator m_linearIntake = new Elevator(m_config);

  public LinearIntakeSubsystem() {
    new Trigger(() -> getHeight().lte(Meters.of(0.1)))
        .and(
            () -> linearIntakeInputs.setpoint.isEquivalent(
                motorConfig.convertFromMechanism(Rotations.of(0))))
        .whileTrue(m_linearIntake.set(0));
  }

  private void updateInputs() {
    linearIntakeInputs.setpoint = motorConfig.convertFromMechanism(
        m_linearIntake.getMechanismSetpoint().orElse(Rotations.of(1)));
    linearIntakeInputs.position = m_linearIntake.getHeight();
    linearIntakeInputs.velocity = m_linearIntake.getVelocity();
    linearIntakeInputs.current = motor.getStatorCurrent();
    linearIntakeInputs.volts = motor.getVoltage();
  }

  public void periodic() {
    updateInputs();
    Logger.processInputs("LinearIntake", linearIntakeInputs);
    m_linearIntake.updateTelemetry();
  }

  public void simulationPeriodic() {
    m_linearIntake.simIterate();
  }

  public Command elevCmd(double dutycycle) {
    Logger.recordOutput("LinearIntake/DutyCycle", dutycycle);
    return m_linearIntake.set(dutycycle);
  }

  public Command setHeight(Distance height) {
    Logger.recordOutput("LinearIntake/Setpoint", height);
    return m_linearIntake.setHeight(height);
  }

  public Command sysId() {
    return m_linearIntake.sysId(Volts.of(12), Volts.of(12).per(Second), Second.of(30));
  }

  public Distance getHeight() {
    return linearIntakeInputs.position;
  }

  public Distance getSetpoint() {
    return linearIntakeInputs.setpoint;
  }

  public Pose3d getPose3d() {
    double angle = Math.toRadians(180 + 24.159);
    double position = linearIntakeInputs.position.in(Meters) * -1 + 0.3132;

    // Convert distance vector to x and z components based on angle
    double x = position * Math.cos(angle);
    double z = position * Math.sin(angle);
    return new Pose3d(new Translation3d(x, 0, z), new Rotation3d());
  }
}
