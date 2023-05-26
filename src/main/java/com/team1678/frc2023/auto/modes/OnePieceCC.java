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
import com.team1678.frc2023.shuffleboard.ShuffleBoardInteractions;
import com.team1678.frc2023.subsystems.Drive;
import com.team1678.frc2023.subsystems.EndEffector;
import com.team1678.frc2023.subsystems.Superstructure;
import com.team1678.frc2023.subsystems.EndEffector.State;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;

public class OnePieceCC extends AutoModeBase {

    private Superstructure mSuperstructure;
    private EndEffector mEffector;

    // required PathWeaver trajectory paths
    String path1 = "paths/CCOnePiece1Deploy.path";
    String path2 = "paths/CCOnePiece2Deploy.path";

    // trajectories
    SwerveTrajectoryAction driveToFirstPickup;
    final Trajectory drive_to_first_pickup_path;

    public OnePieceCC() {

        mSuperstructure = Superstructure.getInstance();
        mEffector = EndEffector.getInstance();

        // read trajectories from PathWeaver and generate trajectory actions
        drive_to_first_pickup_path = AutoTrajectoryReader.generateTrajectoryFromFile(path1,
                Constants.AutoConstants.createConfig(2.0, 10.0, 0.0, 180.0));
        driveToFirstPickup = new SwerveTrajectoryAction(drive_to_first_pickup_path, Rotation2d.fromDegrees(0.0));
        ShuffleBoardInteractions.getInstance().mFieldView.addTrajectory("Traj", drive_to_first_pickup_path);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new LambdaAction(() -> Drive.getInstance().resetOdometry(getStartingPose())));

        System.out.println("Running One Plus Balance CC");

        mSuperstructure.stowState();
        runAction(new WaitForSuperstructureAction());
        System.out.println("Finished waiting for stow");
        mSuperstructure.scoreL3State();
        runAction(new WaitForSuperstructureAction());
        System.out.println("Finished waiting for extend");
        runAction(new WaitAction(0.5));
        mEffector.setState(State.OUTTAKING_CONE);
        runAction(new WaitAction(0.5));
        mEffector.setState(State.IDLE);
        System.out.println("Finished running endeffector");
        mSuperstructure.stowElevator();
        runAction(new WaitForSuperstructureAction());

        runAction(new ParallelAction(List.of(
                driveToFirstPickup,
                new SeriesAction(List.of(
                        new WaitAction(1.0),
                        new LambdaAction(() -> Drive.getInstance()
                                .setAutoHeading(Rotation2d.fromDegrees(0.0))))),
                new SeriesAction(List.of(
                        new LambdaAction(() -> mSuperstructure.stowState()),
                        new WaitAction(1.0),
                        new LambdaAction(() -> mSuperstructure.groundIntakeState()),
                        new LambdaAction(() -> mEffector.setState(State.INTAKING_CUBE)))))));

        runAction(new WaitAction(1.0));
        mSuperstructure.stowWrist();
        mEffector.setState(State.IDLE);
    }

    @Override
    public Pose2d getStartingPose() {
            Rotation2d startingRotation = Rotation2d.fromDegrees(180.0);
            if (Robot.is_red_alliance) {
                    startingRotation = Rotation2d.fromDegrees(0.0);
            }
            return new Pose2d(drive_to_first_pickup_path.getInitialPose().getTranslation(), startingRotation);
    }
}
