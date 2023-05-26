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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;

public class ThreePiece extends AutoModeBase {

    private Superstructure mSuperstructure;
    private EndEffector mEffector;

    // required PathWeaver trajectory paths
    String path1 = "paths/LeftScoreToSwoop.path";
    String path2 = "paths/CubeScoreToMidLeftPickup.path";
    String path3 = "paths/MidLeftPickupToDrop.path";
    String path4 = "paths/DropDriveOut.path";

    // trajectories
    SwerveTrajectoryAction drive_to_intake;
    final Trajectory drive_to_intake_path;

    SwerveTrajectoryAction drive_to_cone;
    final Trajectory drive_to_cone_path;

    SwerveTrajectoryAction drive_to_drop;
    final Trajectory drive_to_drop_path;

    SwerveTrajectoryAction drive_out;
    final Trajectory drive_out_path;

    public ThreePiece() {

        mSuperstructure = Superstructure.getInstance();
        mEffector = EndEffector.getInstance();

        // read trajectories from PathWeaver and generate trajectory actions
        drive_to_intake_path = AutoTrajectoryReader.generateTrajectoryFromFile(path1,
                Constants.AutoConstants.createConfig(3.5, 3.0, 0.0, 0.0));
        drive_to_intake = new SwerveTrajectoryAction(drive_to_intake_path, Rotation2d.fromDegrees(180.0));
        ShuffleBoardInteractions.getInstance().mFieldView.addTrajectory("Traj", drive_to_intake_path);

        // read trajectories from PathWeaver and generate trajectory actions
        drive_to_cone_path = AutoTrajectoryReader.generateTrajectoryFromFile(path2,
                        Constants.AutoConstants.createConfig(3.5, 3.0, 0.0, 0.0));
        drive_to_cone = new SwerveTrajectoryAction(drive_to_cone_path, Rotation2d.fromDegrees(180.0));
        ShuffleBoardInteractions.getInstance().mFieldView.addTrajectory("Traj2", drive_to_cone_path);

                // read trajectories from PathWeaver and generate trajectory actions
        drive_to_drop_path = AutoTrajectoryReader.generateTrajectoryFromFile(path3,
                Constants.AutoConstants.createConfig(3.5, 3.0, 0.0, 0.0));
        drive_to_drop = new SwerveTrajectoryAction(drive_to_drop_path, Rotation2d.fromDegrees(-30.0));
        ShuffleBoardInteractions.getInstance().mFieldView.addTrajectory("Traj3", drive_to_drop_path);
        
        // read trajectories from PathWeaver and generate trajectory actions
        drive_out_path = AutoTrajectoryReader.generateTrajectoryFromFile(path4,
        Constants.AutoConstants.createConfig(4.0, 4.0, 0.0, 0.0));
        drive_out = new SwerveTrajectoryAction(drive_out_path, Rotation2d.fromDegrees(170.0));
        ShuffleBoardInteractions.getInstance().mFieldView.addTrajectory("Traj4", drive_out_path);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Two Piece");
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
                drive_to_intake,
                new SeriesAction(List.of(
                        new LambdaAction(() -> mSuperstructure.stowState()),
                        new WaitToPassXCoordinateAction(4.818),
                        new LambdaAction(() -> Drive.getInstance()
                                                .setAutoHeading(Rotation2d.fromDegrees(200.0))),
                        new LambdaAction(() -> mSuperstructure.groundIntakeFloatState()),
                        new LambdaAction(() -> mEffector.setState(State.INTAKING_CUBE)),
                        new WaitToPassXCoordinateAction(7.0),
                        // new WaitForAquisition(),
                        new WaitToPassXCoordinateAction(6.7),
                        new LambdaAction(() -> mSuperstructure.stowState()),
                        new LambdaAction(() -> Drive.getInstance()
                                                .setAutoHeading(Rotation2d.fromDegrees(180.0))),
                        new LambdaAction(() -> mEffector.setState(State.IDLE)),
                        new LambdaAction(() -> mSuperstructure.scoreStandbyState(true)),
                        new WaitToPassXCoordinateAction(3.0),
                        new LambdaAction(() -> mSuperstructure.scoreL3State())
                ))
        )));
        runAction(new WaitForSuperstructureAction());
        System.out.println("Finished waiting for extend");
        mEffector.setState(State.OUTTAKING_CUBE);
        runAction(new WaitAction(0.2));
        System.out.println("Finished running endeffector");
        mSuperstructure.stowElevator();
        runAction(new WaitForSuperstructureAction());

        // New wait
        runAction(new WaitAction(0.5));

        runAction(new ParallelAction(List.of(
                        drive_to_cone,
                        new SeriesAction(List.of(
                                        new LambdaAction(() -> Drive.getInstance()
                                                .setAutoHeading(Rotation2d.fromDegrees(180.0))),
                                        new LambdaAction(() -> mSuperstructure.stowState()),
                                        new LambdaAction(() -> mEffector.setState(State.INTAKING_CONE)),
                                        new WaitToPassXCoordinateAction(3.5),
                                        new LambdaAction(() -> Drive.getInstance()
                                                        .setAutoHeading(Rotation2d.fromDegrees(-30))),
                                        new WaitToPassXCoordinateAction(4.4),
                                        new LambdaAction(() -> mSuperstructure.groundIntakeState())
        
                        ))
                )));
        mSuperstructure.stowState();
        runAction(new ParallelAction(List.of(
                drive_to_drop,
                new SeriesAction(List.of(
                                new WaitToPassXCoordinateAction(4.6),
                                new LambdaAction(() -> mSuperstructure.scoreL1State()),
                                new LambdaAction(() -> mEffector.setState(State.IDLE)),
                                new WaitToPassXCoordinateAction(2.3),
                                new LambdaAction(() -> mEffector.setState(State.OUTTAKING_CONE))

                )),
                new SeriesAction(List.of(
                        new WaitAction(0.1),
                        new LambdaAction(() -> Drive.getInstance()
                                .setAutoHeading(Rotation2d.fromDegrees(170.0)))
                ))
        )));

        runAction(new ParallelAction(List.of(
                drive_out,
                new LambdaAction(() -> mSuperstructure.stowState())
        )));
    }

    @Override
    public Pose2d getStartingPose() {
            Rotation2d startingRotation = Rotation2d.fromDegrees(180.0);
            Translation2d startingTranslation = drive_to_intake_path.getInitialPose().getTranslation();
            if (Robot.is_red_alliance) {
                    startingRotation = Rotation2d.fromDegrees(0.0);
                //     startingTranslation = startingTranslation.plus(new Translation2d(0, 0.4));
            }
            return new Pose2d(startingTranslation, startingRotation);
    }
}
