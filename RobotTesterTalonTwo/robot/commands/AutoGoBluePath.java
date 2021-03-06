/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import edu.wpi.first.wpilibj2.command.WaitCommand;
//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
//import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
//import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.AutoDriveStraightTime;
import frc.robot.subsystems.DriveTrain;

/**
 * A complex autonomous command that drives forward, and then drives backward.
 */
public class AutoGoBluePath extends SequentialCommandGroup {
  
  /**
   * Creates a new SequentialCommandGroup
   *
   * @param driveTrain The DriveTrain subsystem this command will run on
   */
  public AutoGoBluePath(DriveTrain driveTrain) {

    addCommands(
	    new InstantCommand(() -> driveTrain.stop(), driveTrain),    // make sure stopped
	    new AutoDriveStraightTime( 0.40, 2.0),  // drive forward
      new AutoSpinToAnglePID(45.0, 0.5),      // spin 45 deg
      new AutoDriveStraightTime( 0.40, 2.0),  // drive forward
	    new InstantCommand(driveTrain::stop, driveTrain)       // make sure stopped
    );
  }
}
