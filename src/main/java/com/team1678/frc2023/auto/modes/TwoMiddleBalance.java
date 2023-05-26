package com.team1678.frc2023.auto.modes;

import java.util.List;

import com.team1678.frc2023.Constants;
import com.team1678.frc2023.Robot;
import com.team1678.frc2023.auto.AutoModeBase;
import com.team1678.frc2023.auto.AutoModeEndedException;
import com.team1678.frc2023.auto.AutoTrajectoryReader;
import com.team1678.frc2023.auto.actions.LambdaAction;
import com.team1678.frc2023.auto.actions.ParallelAction;
import com.team1678.frc2023.auto.actions.SeriesAction;
import com.team1678.frc2023.auto.actions.SwerveTrajectoryAction;
import com.team1678.frc2023.auto.actions.WaitAction;
import com.team1678.frc2023.auto.actions.WaitForSuperstructureAction;
import com.team1678.frc2023.auto.actions.WaitToPassXCoordinateAction;
import com.team1678.frc2023.shuffleboard.ShuffleBoardInteractions;
import com.team1678.frc2023.subsystems.Drive;
import com.team1678.frc2023.subsystems.EndEffector;
import com.team1678.frc2023.subsystems.Superstructure;
import com.team1678.frc2023.subsystems.EndEffector.State;
import com.team1678.lib.drivers.Pigeon;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;

public class TwoMiddleBalance extends AutoModeBase {

    private Superstructure mSuperstructure;
    private EndEffector mEffector;

    // required PathWeaver trajectory paths
    String path1 = "paths/CSRightScoreToPickup.path";
    String path2 = "paths/CSRightPickupToScore.path";
    // trajectories
    SwerveTrajectoryAction drive_to_piece;
    final Trajectory drive_to_piece_path;

    SwerveTrajectoryAction drive_to_score;
    final Trajectory drive_to_score_path;

    public TwoMiddleBalance() {
        mSuperstructure = Superstructure.getInstance();
        mEffector = EndEffector.getInstance();


        // read trajectories from PathWeaver and generate trajectory actions
        drive_to_piece_path = AutoTrajectoryReader.generateTrajectoryFromFile(path1,
                Constants.AutoConstants.createConfig(1.5, 8.0, 0.0, 0.0));
        drive_to_piece = new SwerveTrajectoryAction(drive_to_piece_path, Rotation2d.fromDegrees(180.0));
        ShuffleBoardInteractions.getInstance().mFieldView.addTrajectory("Traj 1", drive_to_piece_path);

        // read trajectories from PathWeaver and generate trajectory actions
        drive_to_score_path = AutoTrajectoryReader.generateTrajectoryFromFile(path2,
                Constants.AutoConstants.createConfig(1.5, 8.0, 0.0, 0.0));
        drive_to_score = new SwerveTrajectoryAction(drive_to_score_path, Rotation2d.fromDegrees(0.0));
        ShuffleBoardInteractions.getInstance().mFieldView.addTrajectory("Traj 2", drive_to_score_path);
        
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new LambdaAction(() -> Drive.getInstance().resetOdometry(getStartingPose())));

        // Debug
        System.out.println("Running Two Balance CS");
        System.out.println("Finished waiting for stow");
        mSuperstructure.scoreL3State();
        runAction(new WaitForSuperstructureAction());
        System.out.println("Finished waiting for extend");
        runAction(new WaitAction(0.5));
        mEffector.setState(State.OUTTAKING_CONE);
        runAction(new WaitAction(0.3));
        mEffector.setState(State.IDLE);
        System.out.println("Finished running endeffector");
        mSuperstructure.stowElevator();
        runAction(new WaitForSuperstructureAction());

        runAction(new ParallelAction(List.of(
            drive_to_piece,
            new SeriesAction(List.of(
                    new LambdaAction(() -> mSuperstructure.stowState()),
                    new LambdaAction(() -> Drive.getInstance()
                            .setAutoHeading(Rotation2d.fromDegrees(270.0))),
                    new WaitToPassXCoordinateAction(5.600),
                    new LambdaAction(() -> Drive.getInstance()
                            .setAutoHeading(Rotation2d.fromDegrees(0.0))))),
            new SeriesAction(List.of (
                    new WaitAction(2.60),
                    new LambdaAction(() -> mSuperstructure.groundIntakeState()),
                    new LambdaAction(() -> mEffector.setState(State.INTAKING_CUBE)))
        ))));
                    
        runAction(new ParallelAction(List.of(
            drive_to_score,
            new SeriesAction(List.of(
                new LambdaAction(() -> mEffector.setState(State.IDLE)),
                new LambdaAction(() -> mSuperstructure.stowState()),
                new LambdaAction(() -> Drive.getInstance()
                        .setAutoHeading(Rotation2d.fromDegrees(270.0))),
                new WaitToPassXCoordinateAction(3.50),
                new LambdaAction(() -> Drive.getInstance()
                        .setAutoHeading(Rotation2d.fromDegrees(160.0))),
                new WaitToPassXCoordinateAction(3.0),
                new LambdaAction(() -> mSuperstructure.scoreL1State()),
                new WaitForSuperstructureAction(),
                new WaitAction(0.5),
                new LambdaAction(() -> mEffector.setState(State.OUTTAKING_CUBE))
        )))));

        runAction(new WaitAction(0.25));
        mEffector.setState(State.IDLE);
        mSuperstructure.stowState(); 
        Pigeon.getInstance().setPitch(0.0);
        // runAction(new WaitForSuperstructureAction());
        mSuperstructure.autoBalance();
        new WaitForSuperstructureAction();
        System.out.println("Auto is finished!");
    }

    @Override
    public Pose2d getStartingPose() {
        Rotation2d startingRotation = Rotation2d.fromDegrees(180.0);
        if (Robot.is_red_alliance) {
            startingRotation = Rotation2d.fromDegrees(0.0);
        }
        return new Pose2d(drive_to_piece_path.getInitialPose().getTranslation(), startingRotation);
    }
}
