/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import edu.wpi.first.wpilibj2.command.WaitCommand;
//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
//import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
//import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.AutoDriveStraightTime;
import frc.robot.commands.AutoSpinToAnglePID;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoGoBluePath2 extends SequentialCommandGroup {
  
  /**
   * Creates a new SequentialCommandGroup
   *
   * @param driveTrain The DriveTrain subsystem this command will run on
   */
  public AutoGoBluePath2(DriveTrain driveTrain) {
 
    // positive angles turn right, negative angles turn left
    addCommands(
	    new InstantCommand(() -> driveTrain.stop(), driveTrain),    // make sure stopped
      new AutoSpinToAnglePID(40, 0.6),        // turn to 2nd ball
      new AutoDriveStraightTime( -0.6, 1.5),  // go to 2nd ball
      new AutoSpinToAnglePID(-75, 0.6),       // turn to 3rd ball
      new AutoDriveStraightTime( -0.6, 2.0),  // go to 3rd ball
      new AutoSpinToAnglePID(45, 0.6),        // turn to end zone
      new AutoDriveStraightTime( -0.6, 3.0),  // go to end zone
      new InstantCommand(driveTrain::stop, driveTrain)       // make sure stopped
    );

      /*
      // additional complex commands
      //new ParallelCommandGroup(new AutoDriveStraightTime( 0.45, 2.0), new WaitCommand(3.0)),
      //new ParallelDeadlineGroup(new AutoTurnToAngle(45.0, 0.5), new WaitCommand(3.0)),
      //new AutoDriveStraightTime( 0.45, 2.0),
      //new SequentialCommandGroup(new AutoSpinToAngle( -45.0, 2.0), new WaitCommand(2.0)),
      //new InstantCommand(driveTrain::stop, driveTrain)
      */
  }
}
