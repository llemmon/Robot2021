/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
//import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
//import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.AutoDriveStraightTime;
import frc.robot.commands.AutoSpinToAnglePID;
import frc.robot.subsystems.DriveTrain;

/**
 * A complex autonomous command that drives forward, and then drives backward.
 */
public class AutoGoBouncePath extends SequentialCommandGroup {

  /**
   * Creates a new SequentialCommandGroup
   *
   * @param driveTrain The DriveTrain subsystem this command will run on
   */
  public AutoGoBouncePath(DriveTrain driveTrain) {

    addCommands(
      new InstantCommand(() -> driveTrain.stop(), driveTrain),
      new AutoDriveStraightTime( 0.5, 2.5),
      new AutoSpinToAnglePID(-58.0, 0.4),
      new AutoDriveStraightTime(0.5, 2.0),
      new AutoDriveStraightTime(-0.5, 1.5),
      new AutoSpinToAnglePID(-30.0, 0.4),
      new AutoDriveStraightTime(-0.5, 4.0),
      new AutoSpinToAnglePID(60.0, 0.4),//
      new WaitCommand(0.25),
      new AutoDriveStraightTime(0.5, 2),
      new WaitCommand(0.25),
      new AutoSpinToAnglePID(-50.0, 0.4),
      new WaitCommand(0.25),
      new AutoDriveStraightTime(0.5, 3.8),
      new WaitCommand(0.25),
      new AutoDriveStraightTime(-0.5, 4.5),
      new WaitCommand(0.25),
      new AutoSpinToAnglePID(50, 0.4),
      new WaitCommand(0.25),
      new AutoDriveStraightTime(0.5, 5),
      new WaitCommand(0.25),
      new AutoSpinToAnglePID(-63.0, 0.4),
      new WaitCommand(0.25),
      new AutoDriveStraightTime(0.5, 5),
      new WaitCommand(0.25),
      new AutoDriveStraightTime(-0.5, 2),
      new WaitCommand(0.25),
      new AutoSpinToAnglePID(80.0, 0.4),
      new WaitCommand(0.25),
      new AutoDriveStraightTime(0.5, 3),
      new WaitCommand(0.25),
      new InstantCommand(driveTrain::stop, driveTrain)
); //Positive is Right , Negative is Leftt

  //super(new InstantCommand(() -> driveTrain.stop(), driveTrain),
  //new ParallelCommandGroup(new AutoDriveStraightTime( 0.45, 2.0), new WaitCommand(3.0)),
  //new ParallelDeadlineGroup(new AutoTurnToAngle(45.0, 0.5), new WaitCommand(3.0)),
  //new AutoDriveStraightTime( 0.45, 2.0),
  //new SequentialCommandGroup(new AutoSpinToAngle( -45.0, 2.0), new WaitCommand(2.0)),
  //new InstantCommand(driveTrain::stop, driveTrain)
  }
}
