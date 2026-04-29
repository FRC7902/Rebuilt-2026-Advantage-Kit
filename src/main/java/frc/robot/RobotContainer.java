package frc.robot;

import frc.robot.subsystems.*;

public class RobotContainer {

  private final ArmSubsystem arm;
  private final ElevatorSubsystem elevator;
  private final ShooterSubsystem shooter;

  public RobotContainer() {
    arm = new ArmSubsystem(1);
    elevator = new ElevatorSubsystem(2);
    shooter = new ShooterSubsystem(3);

    configureBindings();
  }

  private void configureBindings() {}
}
