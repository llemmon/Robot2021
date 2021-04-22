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

    // positive angles turn right, negative angles turn left
    addCommands(
	    new InstantCommand(() -> driveTrain.stop(), driveTrain),    // make sure stopped
      new AutoDriveStraightTime(0.5, 5.5), //forward DONE
      new AutoSpinToAnglePID(35.0, 0.50), //turn down DONE
      new AutoDriveStraightTime(0.50, 3.0), //left
      new AutoSpinToAnglePID( -55.0, -0.45), //Temp
      new AutoDriveStraightTime(0.60, 2.0), // up
      new AutoSpinToAnglePID(-30.0, 0.50), // turn right 
      new AutoDriveStraightTime(0.50, 4.0),

      /* Temp
      // forward but up a bit 

      new AutoDriveStraightTime(0.50, 2.0), //up  
      new AutoSpinToAnglePID(45.0, 0.50), // turn left 
      new AutoSpinToAnglePID(30.0, 0.50), // turn down  
      new AutoDriveStraightTime(0.50, 2.0), // go down  
      new AutoSpinToAnglePID(-30.0, 0.50), // turn right a bit 
      new AutoDriveStraightTime(0.50, 2.0), // up 
      new AutoSpinToAnglePID(45.0, 0.50), // turn left */ 
      
      new InstantCommand(driveTrain::stop, driveTrain)       // make sure stopped
    );
  }
}
