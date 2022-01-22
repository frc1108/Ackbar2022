package frc.robot.commands.auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class PickupOne extends SequentialCommandGroup {
  public PickupOne(DriveSubsystem m_robotDrive) {        
      TrajectoryConfig config = new TrajectoryConfig(2, 3);
      
      Trajectory trajToCargo = m_robotDrive.generateTrajectory("Barrel1", config);
      
      addCommands(
          new InstantCommand(() -> {
              m_robotDrive.resetOdometry(trajToCargo.getInitialPose());
          }),
          m_robotDrive.createCommandForTrajectory(trajToCargo, false).withTimeout(10).withName("DriveToCargo")
      );
  }
}