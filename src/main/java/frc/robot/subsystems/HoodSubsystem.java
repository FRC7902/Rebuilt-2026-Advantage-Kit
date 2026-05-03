package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class HoodSubsystem extends SubsystemBase {

  /**
   * AdvantageKit identifies inputs via the "Replay Bubble". Everything going to the SMC is an
   * Output. Everything coming from the SMC is an Input.
   */
  @AutoLog
  public static class HoodInputs {
    public Angle pivotPosition = Degrees.of(0);
    public AngularVelocity pivotVelocity = DegreesPerSecond.of(0);
    public Angle pivotDesiredPosition = Degrees.of(0);
    public Voltage pivotAppliedVolts = Volts.of(0);
    public Current pivotCurrent = Amps.of(0);
  }

  private final HoodInputsAutoLogged hoodInputs = new HoodInputsAutoLogged();

  private final TalonFX motor;

  // YAMS Configurations
  private final SmartMotorControllerConfig smcConfig;
  private final SmartMotorController smc;
  private final ArmConfig hoodCfg;

  // Arm Mechanism
  private final Arm hood;

  public HoodSubsystem() {
    motor = new TalonFX(HoodConstants.CAN_ID);
    smcConfig = HoodConstants.SMC_CONFIG.withSubsystem(this);
    smc = new TalonFXWrapper(motor, HoodConstants.MOTOR_TYPE, smcConfig);
    hoodCfg = HoodConstants.ARM_CONFIG.withSmartMotorController(smc);
    hood = new Arm(hoodCfg);
  }

  /** Updates AdvantageKit inputs from the {@link Arm} to be used in the rest of the program. */
  public void updateInputs() {
    hoodInputs.pivotPosition = hood.getAngle();
    hoodInputs.pivotVelocity = smc.getMechanismVelocity();
    hoodInputs.pivotAppliedVolts = smc.getVoltage();
    hoodInputs.pivotCurrent = smc.getStatorCurrent();
  }

  /**
   * Set the angle of the hood.
   *
   * @param angle Angle to go to.
   */
  public Command setAngle(Angle angle) {
    return hood.setAngle(angle);
  }

  /**
   * Move the hood up and down.
   *
   * @param dutyCycle [-1, 1] speed to set the hood too.
   */
  public Command set(double dutyCycle) {
    return hood.set(dutyCycle);
  }

  /** Run sysId on the {@link Arm} */
  public Command sysId() {
    return hood.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateInputs();
    Logger.processInputs("Hood", hoodInputs);
    hood.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    hood.simIterate();
  }

  @AutoLogOutput
  public Angle getAngleSetpoint() {
    return smc.getMechanismPositionSetpoint().orElse(Degrees.of(0));
  }

  public Angle getAngle() {
    return hoodInputs.pivotPosition;
  }

  public AngularVelocity getVelocity() {
    return hoodInputs.pivotVelocity;
  }

  public Angle getSetpointAngle() {
    return hoodInputs.pivotDesiredPosition;
  }

  public Voltage getVoltage() {
    return hoodInputs.pivotAppliedVolts;
  }

  public Current getCurrent() {
    return hoodInputs.pivotCurrent;
  }

  public Pose3d getPose3d() {
    return new Pose3d(
        new Translation3d(0.2286, 0, 0.4572),
        new Rotation3d(0, hoodInputs.pivotPosition.in(Radians), 0));
  }
}
