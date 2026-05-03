package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.LinearIntakeConstants;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

/** AdvantageKit Elevator Subsystem, capable of replaying the elevator. */
public class LinearIntakeSubsystem extends SubsystemBase {

  /**
   * AdvantageKit identifies inputs via the "Replay Bubble". Everything going to the SMC is an
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

  private final LinearIntakeInputsAutoLogged linearIntakeInputs =
      new LinearIntakeInputsAutoLogged();

  private final SparkMax leaderMotor =
      new SparkMax(LinearIntakeConstants.LEADER_CAN_ID, SparkLowLevel.MotorType.kBrushless);
  private final SparkMax followerMotor =
      new SparkMax(LinearIntakeConstants.FOLLOWER_CAN_ID, SparkLowLevel.MotorType.kBrushless);

  private final SmartMotorControllerConfig smcConfig;
  private final SmartMotorController smc;
  private ElevatorConfig linearIntakeConfig;
  private final Elevator linearIntake;

  public LinearIntakeSubsystem() {
    smcConfig =
        LinearIntakeConstants.SMC_CONFIG
            .withSubsystem(this)
            // LinearIntake motor2 follows LinearIntake motor with an inversed output.
            .withFollowers(Pair.of(followerMotor, LinearIntakeConstants.MOTOR2_INVERTED));
    smc = new SparkWrapper(leaderMotor, LinearIntakeConstants.MOTOR_TYPE, smcConfig);
    linearIntakeConfig = LinearIntakeConstants.ELEVATOR_CONFIG.withSmartMotorController(smc);
    linearIntake = new Elevator(linearIntakeConfig);

    new Trigger(() -> getHeight().lte(Meters.of(0.1)))
        .and(
            () ->
                linearIntakeInputs.setpoint.isEquivalent(
                    smcConfig.convertFromMechanism(Rotations.of(0))))
        .whileTrue(linearIntake.set(0));
  }

  private void updateInputs() {
    linearIntakeInputs.setpoint =
        smcConfig.convertFromMechanism(linearIntake.getMechanismSetpoint().orElse(Rotations.of(0)));
    linearIntakeInputs.position = linearIntake.getHeight();
    linearIntakeInputs.velocity = linearIntake.getVelocity();
    linearIntakeInputs.current = smc.getStatorCurrent();
    linearIntakeInputs.volts = smc.getVoltage();
  }

  @Override
  public void periodic() {
    updateInputs();
    Logger.processInputs("LinearIntake", linearIntakeInputs);
    linearIntake.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    linearIntake.simIterate();
  }

  public Command set(double dutyCycle) {
    Logger.recordOutput("LinearIntake/DutyCycle", dutyCycle);
    return linearIntake.set(dutyCycle);
  }

  public Command setHeight(Distance height) {
    Logger.recordOutput("LinearIntake/Setpoint", height);
    return linearIntake.setHeight(height);
  }

  public Command sysId() {
    return linearIntake.sysId(Volts.of(12), Volts.of(12).per(Second), Second.of(30));
  }

  public Distance getHeight() {
    return linearIntakeInputs.position;
  }

  public Distance getSetpoint() {
    return linearIntakeInputs.setpoint;
  }

  public Pose3d getPose3d() {
    double angle = Math.toRadians(LinearIntakeConstants.STARTING_ANGLE);
    double position =
        linearIntakeInputs.position.in(Meters) * -1 + LinearIntakeConstants.MAX_DISTANCE.in(Meters);

    // Convert distance vector to x and z components based on angle
    double x = position * Math.cos(angle);
    double z = position * Math.sin(angle);
    return new Pose3d(new Translation3d(x, 0, z), new Rotation3d());
  }
}
