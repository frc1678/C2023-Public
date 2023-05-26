package com.team1678.frc2023.auto.modes;

import com.team1678.frc2023.Constants;
import com.team1678.frc2023.auto.AutoModeBase;
import com.team1678.frc2023.auto.AutoModeEndedException;
import com.team1678.frc2023.auto.AutoTrajectoryReader;
import com.team1678.frc2023.auto.actions.LambdaAction;
import com.team1678.frc2023.auto.actions.SwerveTrajectoryAction;
import com.team1678.frc2023.auto.actions.WaitAction;
import com.team1678.frc2023.shuffleboard.ShuffleBoardInteractions;
import com.team1678.frc2023.subsystems.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;

public class TestPathMode extends AutoModeBase {
    
    // Swerve instance 
    private final Drive mSwerve = Drive.getInstance();

    // required PathWeaver trajectory paths
    String path1 = "paths/TestPaths/TestPath1.path";
    String path2 = "paths/TestPaths/TestPath2.path";
    
	// trajectories
	SwerveTrajectoryAction driveAction1;
    final Trajectory trajectory1;

    SwerveTrajectoryAction driveAction2;
    final Trajectory trajectory2;

    public TestPathMode() {

        // read trajectories from PathWeaver and generate trajectory actions
        trajectory1 = AutoTrajectoryReader.generateTrajectoryFromFile(path1, Constants.AutoConstants.createConfig(4.8, 1.8, 0.0, 0.0));
        driveAction1 = new SwerveTrajectoryAction(trajectory1, Rotation2d.fromDegrees(0.0));
		ShuffleBoardInteractions.getInstance().mFieldView.addTrajectory("Traj", trajectory1);

        trajectory2 = AutoTrajectoryReader.generateTrajectoryFromFile(path2,
                Constants.AutoConstants.createConfig(4.8, 1.8, 0.0, 0.0));
        driveAction2 = new SwerveTrajectoryAction(trajectory2, Rotation2d.fromDegrees(0.0));
        ShuffleBoardInteractions.getInstance().mFieldView.addTrajectory("Traj2", trajectory2);

        mSwerve.resetOdometry(getStartingPose());
    }

    @Override
    protected void routine() throws AutoModeEndedException {

        System.out.println("Running test mode auto!");

        runAction(new LambdaAction(() -> mSwerve.resetOdometry(getStartingPose())));

        runAction(new WaitAction(1.0));

        runAction(driveAction1);
        System.out.println(Timer.getFPGATimestamp() + " finished running path 1");
        // runAction(driveAction2);
        
        System.out.println("Finished auto!");
    }

    @Override
    public Pose2d getStartingPose() {
        Rotation2d startingRotation = Rotation2d.fromDegrees(0.0);
        // if (Robot.is_red_alliance) {
        //     startingRotation = Rotation2d.fromDegrees(0.0);
        // }
        return new Pose2d(trajectory1.getInitialPose().getTranslation(), startingRotation);
    }
}


