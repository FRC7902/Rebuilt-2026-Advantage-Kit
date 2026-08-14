package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LinearIntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import java.io.File;
import org.littletonrobotics.junction.Logger;
import swervelib.SwerveInputStream;

public class RobotContainer {

  private final HoodSubsystem hood;
  private final LinearIntakeSubsystem linearIntake;
  private final ShooterSubsystem shooter;
  private final IndexerSubsystem indexer;

  final CommandPS4Controller driverController = new CommandPS4Controller(0);
  private final SwerveSubsystem swerve =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  // Establish a Sendable Chooser that will be able to be sent to the
  // SmartDashboard, allowing selection of desired auto
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular
   * velocity.
   */
  SwerveInputStream driveAngularVelocity =
      SwerveInputStream.of(
              swerve.getSwerveDrive(),
              () -> driverController.getLeftY() * -1,
              () -> driverController.getLeftX() * -1)
          .withControllerRotationAxis(() -> driverController.getRightX() * -1)
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);

  public RobotContainer() {
    hood = new HoodSubsystem();
    linearIntake = new LinearIntakeSubsystem();
    shooter = new ShooterSubsystem();
    indexer = new IndexerSubsystem();

    DriverStation.silenceJoystickConnectionWarning(true);

    configureBindings();
  }

  public void publishComponentPoses() {
    Logger.recordOutput(
        "3D/ComponentPoses", new Pose3d[] {linearIntake.getPose3d(), hood.getPose3d()});
  }

  Command printDebug(String out) {
    return Commands.runOnce(() -> System.out.println(out));
  }

  private void configureBindings() {
    Command driveFieldOrientedAnglularVelocity = swerve.driveFieldOriented(driveAngularVelocity);
    swerve.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    System.out.println("yoohoo");

    // the euhhh first one whre you press once
    // i had to comment the trigger in LinearIntakeSubsystem.java because otherwise thered be a bug where the intake would only reset on the next cycle, and
    // get stuck on max extension on the first one. checking AdvantageScope it reaches the targets fine
    driverController
        .button(1)
        .onTrue(
            Commands.sequence(
                Commands.parallel(
                    Commands.waitSeconds(5.0),
                    indexer.set(0.5),
                    linearIntake.setHeightAndStop(Meters.of(0))),
                linearIntake.setHeightAndStop(Meters.of(0.3132)),
                indexer.set(0.0)));

    // the other one
    driverController
        .button(2)
        .whileTrue(
            Commands.sequence(
                Commands.parallel(
                    hood.setAngle(Degrees.of(20)),
                    shooter.setVelocity(RPM.of(2500)),
                    Commands.sequence(
                        Commands.waitUntil(
                            () -> hood.getAngle().isNear(Degrees.of(20), Degrees.of(0.5))),
                        indexer.set(0.2)))))
        .onFalse(
            Commands.parallel(
                shooter.setVelocity(RPM.of(0)), hood.setAngle(Degrees.of(0)), indexer.set(0.0)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Pass in the selected auto from the SmartDashboard as our desired autnomous
    // commmand
    return autoChooser.getSelected();
  }
}
