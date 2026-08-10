// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.RobotBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.math.ExponentialProfilePIDController;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.config.FlyWheelConfig;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final double MAX_SPEED = Units.feetToMeters(14.5);

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

  public static class HoodConstants {
    public static final int CAN_ID = 40;
    public static final DCMotor MOTOR_TYPE = DCMotor.getFalcon500(1);

    public static final SmartMotorControllerConfig SMC_CONFIG =
        new SmartMotorControllerConfig()
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withClosedLoopController(18, 0, 0.2)
            .withTrapezoidalProfile(DegreesPerSecond.of(458), DegreesPerSecondPerSecond.of(688))
            .withSimClosedLoopController(15.0, 0, 1.0)
            .withFeedforward(new ArmFeedforward(-0.1, 0.9, 0, 0))
            .withSimFeedforward(new ArmFeedforward(-0.1, 0.9, 0, 0))
            .withTelemetry("HoodMotor", TelemetryVerbosity.HIGH)
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(12.5, 1)))
            .withMotorInverted(false)
            .withIdleMode(MotorMode.BRAKE)
            .withStatorCurrentLimit(Amps.of(120));

    public static final ArmConfig ARM_CONFIG =
        new ArmConfig()
            .withHardLimit(Degrees.of(0), Degrees.of(40))
            .withStartingPosition(Degrees.of(0))
            .withLength(Feet.of((14.0 / 12)))
            .withMOI(KilogramSquareMeters.of(0.1055457256))
            .withTelemetry("HoodMech", TelemetryVerbosity.HIGH);

    public static final Angle TOLERANCE = Degrees.of(0.2);
  }

  public static class LinearIntakeConstants {
    public static final int LEADER_CAN_ID = 30;
    public static final int FOLLOWER_CAN_ID = 31;
    public static final DCMotor MOTOR_TYPE = DCMotor.getNEO(1);
    public static final boolean MOTOR2_INVERTED = true;

    private static final Distance CHAIN_PITCH = Inches.of(0.25);
    private static final int TOOTH_COUNT = 22;
    private static final Distance CIRCUMFERENCE = CHAIN_PITCH.times(TOOTH_COUNT);
    private static final Distance RADIUS = CIRCUMFERENCE.div(2 * Math.PI);
    private static final Mass WEIGHT = Pounds.of(16);
    private static final MechanismGearing GEARING =
        new MechanismGearing(GearBox.fromReductionStages(3, 4));

    public static final SmartMotorControllerConfig SMC_CONFIG =
        new SmartMotorControllerConfig()
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withMechanismCircumference(Inches.of(0.25).times(22)) // chain pitch * tooth count
            .withClosedLoopController(30, 0, 0)
            .withSimClosedLoopController(30.0, 0, 0)
            .withExponentialProfile(
                ExponentialProfilePIDController.createElevatorConstraints(
                    Volts.of(12), MOTOR_TYPE, WEIGHT, RADIUS, GEARING))
            .withFeedforward(new ElevatorFeedforward(0, 0.1, 0, 0))
            .withSimFeedforward(new ElevatorFeedforward(0, 0.61, 0, 0))
            .withStatorCurrentLimit(Amps.of(40))
            .withMotorInverted(false)
            .withSoftLimit(Meters.of(0), Meters.of(2))
            .withGearing(GEARING)
            .withIdleMode(MotorMode.BRAKE)
            .withTelemetry("LinearIntakeMotor", TelemetryVerbosity.HIGH);

    public static final Distance MAX_DISTANCE = Meters.of(0.3132);
    public static final double STARTING_ANGLE = 180 + 24.159;

    public static final ElevatorConfig ELEVATOR_CONFIG =
        new ElevatorConfig()
            .withStartingHeight(MAX_DISTANCE)
            .withHardLimits(Meters.of(0), MAX_DISTANCE)
            .withTelemetry("LinearIntakeMech", TelemetryVerbosity.HIGH)
            .withAngle(Degrees.of(STARTING_ANGLE))
            .withMass(WEIGHT);

    public static final Distance TOLERANCE = Meters.of(0.025);
  }

  public static class ShooterConstants {
    public static final int CAN_ID = 20;
    public static final DCMotor MOTOR_TYPE = DCMotor.getNEO(1);

    public static final SmartMotorControllerConfig SMC_CONFIG =
        new SmartMotorControllerConfig()
            .withClosedLoopController(1, 0, 0)
            .withTrapezoidalProfile(RPM.of(10000), RPM.per(Second).of(60))
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
            .withIdleMode(MotorMode.COAST)
            .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
            .withStatorCurrentLimit(Amps.of(40))
            .withMotorInverted(false)
            .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
            .withControlMode(ControlMode.CLOSED_LOOP);
    ;

    public static final FlyWheelConfig FLYWHEEL_CONFIG =
        new FlyWheelConfig()
            // Diameter of the flywheel.
            .withDiameter(Inches.of(4))
            // Mass of the flywheel.
            .withMass(Pounds.of(1))
            .withTelemetry("ShooterMech", TelemetryVerbosity.HIGH);
  }
}
