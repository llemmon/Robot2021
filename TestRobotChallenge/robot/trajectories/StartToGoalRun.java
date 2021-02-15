/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.trajectories;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;

/**
 * Add your docs here.
 */
public class StartToGoalRun {

    private static Trajectory mTrajectory;

    public static Trajectory generateTrajectory() {
        var trajectoyWaypoints = new ArrayList<Pose2d>();

        var start = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        trajectoyWaypoints.add(start);

        //170 to trench
        var end = new Pose2d(Units.inchesToMeters(90), Units.inchesToMeters(-9), Rotation2d.fromDegrees(0));
        trajectoyWaypoints.add(end);

        TrajectoryConfig config = new TrajectoryConfig(1, 1);
        config.setReversed(false);

        var trajectory = TrajectoryGenerator.generateTrajectory(trajectoyWaypoints, config);

        mTrajectory = trajectory;
        return mTrajectory;
    }

    public static Trajectory getTrajectory() { 
        if (mTrajectory == null) {
            return generateTrajectory();
        }
        return mTrajectory;
    }
}
