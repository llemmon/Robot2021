/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
//import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.AutoDriveStraightTime;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class PickBlueBalls extends SequentialCommandGroup {

  public PickBlueBalls(DriveTrain driveTrain) {

    super(new InstantCommand(() -> driveTrain.stop(), driveTrain),
    new ParallelCommandGroup(new AutoDriveStraightTime( 0.45, 3.0), new WaitCommand(4.0)),
    new SequentialCommandGroup(new AutoDriveStraightTime(-0.45, 3.0), new WaitCommand(2.0)),
    new InstantCommand(driveTrain::stop, driveTrain)
    );

    /*
    // alternate way to create command group
    addCommands(
	    new InstantCommand(() -> driveTrain.stop(), driveTrain),
	    // Drive forward the specified distance
	    new AutoDriveStraightTime( 0.45, 3.0),
	    new WaitCommand(2.0),
	    // Drive backward the specified distance
	    new AutoDriveStraightTime( -0.45, 3.0),
	    new InstantCommand(driveTrain::stop, driveTrain),
	    new WaitCommand(2.0)
    );
    */
  }
}
