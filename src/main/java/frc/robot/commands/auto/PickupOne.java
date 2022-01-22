package frc.robot.commands.auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class PickupOne extends SequentialCommandGroup {
  public PickupOne(DriveSubsystem m_robotDrive) {        
      TrajectoryConfig fwdConfig = new TrajectoryConfig(2, 3);
      TrajectoryConfig revConfig = new TrajectoryConfig(2, 3).setReversed(true);
      
      Trajectory trajToCargo = m_robotDrive.generateTrajectory("Cargo1", fwdConfig);
      Trajectory reverseTurn = m_robotDrive.generateTrajectory("Cargo2", revConfig);
      Trajectory terminalRun = m_robotDrive.generateTrajectory("Cargo3", fwdConfig);
      Trajectory returnToHub = m_robotDrive.generateTrajectory("Cargo4", revConfig);
      
      addCommands(
          new InstantCommand(() -> {
              m_robotDrive.resetOdometry(trajToCargo.getInitialPose());
          }),
          m_robotDrive.createCommandForTrajectory(trajToCargo, false).withTimeout(5).withName("Cargo One Pickup"),
          m_robotDrive.createCommandForTrajectory(reverseTurn, false).withTimeout(5).withName("Reverse Turn"),
          m_robotDrive.createCommandForTrajectory(terminalRun, false).withTimeout(5).withName("Terminal Run"),
          m_robotDrive.createCommandForTrajectory(returnToHub, false).withTimeout(5).withName("Return To Hub")
      );
  }
}
