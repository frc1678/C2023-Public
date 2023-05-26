package com.team1678.frc2023.auto.modes;

import com.team1678.frc2023.Constants;
import com.team1678.frc2023.Robot;
import com.team1678.frc2023.auto.AutoModeBase;
import com.team1678.frc2023.auto.AutoModeEndedException;
import com.team1678.frc2023.auto.AutoTrajectoryReader;
import com.team1678.frc2023.auto.actions.LambdaAction;
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

public class OnePiece extends AutoModeBase {

    private Superstructure mSuperstructure;
    private EndEffector mEffector;

    // required PathWeaver trajectory paths
    String path = "paths/OnePlusBalanceDeploy.path";

    // trajectories
    SwerveTrajectoryAction driveToChargeStation;
    final Trajectory drive_to_charge_station_path;

    public OnePiece() {
        mSuperstructure = Superstructure.getInstance();
        mEffector = EndEffector.getInstance();
        
        // read trajectories from PathWeaver and generate trajectory actions
        drive_to_charge_station_path = AutoTrajectoryReader.generateTrajectoryFromFile(path,
                Constants.AutoConstants.createConfig(2.0, 10.0, 0.0, 0.0));
        driveToChargeStation = new SwerveTrajectoryAction(drive_to_charge_station_path, Rotation2d.fromDegrees(0.0));
        ShuffleBoardInteractions.getInstance().mFieldView.addTrajectory("Traj", drive_to_charge_station_path);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new LambdaAction(() -> Drive.getInstance().resetOdometry(getStartingPose())));

        // Debug
        System.out.println("Running One Plus Balance");
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
        runAction(driveToChargeStation);

    }

    @Override
    public Pose2d getStartingPose() {
        Rotation2d startingRotation = Rotation2d.fromDegrees(180.0);
        if (Robot.is_red_alliance) {
            startingRotation = Rotation2d.fromDegrees(0.0);
        }
        return new Pose2d(drive_to_charge_station_path.getInitialPose().getTranslation(), startingRotation);
    }
}
