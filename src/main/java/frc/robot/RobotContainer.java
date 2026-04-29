package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.HoodSubsystem;
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

  final CommandXboxController driverXbox = new CommandXboxController(0);
  private final SwerveSubsystem drivebase =
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
              drivebase.getSwerveDrive(),
              () -> driverXbox.getLeftY() * -1,
              () -> driverXbox.getLeftX() * -1)
          .withControllerRotationAxis(driverXbox::getRightX)
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);

  public RobotContainer() {
    hood = new HoodSubsystem(1);
    linearIntake = new LinearIntakeSubsystem(2);
    shooter = new ShooterSubsystem(3);

    configureBindings();

    // TODO: Remove this after zeroing 3D components
    Logger.recordOutput("3D/RobotPose", new Pose2d());
    Logger.recordOutput("3D/ZeroedComponentPoses", new Pose3d[] {new Pose3d(), new Pose3d()});
  }

  public void publishComponentPoses() {
    Logger.recordOutput(
        "3D/ComponentPoses",
        new Pose3d[] {
          linearIntake.getPose3d(), hood.getPose3d(),
          // shooter.getPose3d()
        });
  }

  private void configureBindings() {
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
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
