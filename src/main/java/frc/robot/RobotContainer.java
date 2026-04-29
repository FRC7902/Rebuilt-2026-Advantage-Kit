package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.subsystems.LinearIntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import java.io.File;

public class RobotContainer {

  private final LinearIntakeSubsystem linearIntakeSubsystem = new LinearIntakeSubsystem();

  public final SwerveSubsystem m_swerveSubsystem =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  public RobotContainer() {}
}
