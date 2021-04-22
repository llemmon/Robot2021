/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
//import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
//import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.AutoDriveStraightTime;
import frc.robot.subsystems.DriveTrain;

public class AutoTurnASquare extends SequentialCommandGroup {

  /**
   * Creates a new SequentialCommandGroup
   *
   * @param driveTrain The DriveTrain subsystem this command will run on
   */
  public AutoTurnASquare(DriveTrain driveTrain) {

    addCommands(
	    new InstantCommand(() -> driveTrain.stop(), driveTrain),
      new AutoDriveStraightTime(0.45, 2.0).andThen(new WaitCommand(0.4)), // go forward
      new AutoSpinToAngle(90.0, 0.45).andThen(new WaitCommand(0.4)),       // turn right
      new AutoDriveStraightTime(0.45, 2.0).andThen(new WaitCommand(0.4)),   // go forward
      new AutoSpinToAngle(90.0, 0.45).andThen(new WaitCommand(0.4)),       // turn right
      new WaitCommand(2.0),
      new AutoDriveStraightTime(0.45, 2.0).andThen(new WaitCommand(0.4)),   // go forward
      new AutoSpinToAngle(90.0, 0.45).andThen(new WaitCommand(0.4)),       // turn right
      new AutoDriveStraightTime(0.45, 2.0),    // go forward to starting point
      new InstantCommand(driveTrain::stop, driveTrain)
    );

      /*
      // alternate way to do a square
      for (int i = 0; i < 4; i++) {
        addCommands(new AutoDriveStraightTime(0.45, 2.0).andThen(new WaitCommand(0.4)));  // repeat 4 times
        addCommands(new AutoSpinToAngle(-90.0, 0.40).andThen(new WaitCommand(0.4)));
      }
      */
  }
}
