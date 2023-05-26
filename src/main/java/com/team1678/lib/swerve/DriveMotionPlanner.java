package com.team1678.lib.swerve;

import java.util.ArrayList;

import com.team1678.frc2023.Constants;
import com.team1678.frc2023.Robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveMotionPlanner {
    private final PIDController forwardController;
    private final PIDController strafeController;
    private final ProfiledPIDController rotationController;

    private final PIDController snapController;
    
    private final HolonomicDriveController mDriveController;

    private Trajectory mCurrentTrajectory;
    private Rotation2d mTargetRotation;
    private Double mStartTime = Double.NaN;

    private boolean isFinished = false;

    public DriveMotionPlanner() {
        forwardController = new PIDController(Constants.AutoConstants.kPXController, 0.0, Constants.AutoConstants.kDXController);
        strafeController = new PIDController(Constants.AutoConstants.kPYController, 0.0, Constants.AutoConstants.kDYController);
        rotationController = new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0.0, 0.0, Constants.AutoConstants.kThetaControllerConstraints);
        snapController = new PIDController(Constants.SnapConstants.kP, Constants.SnapConstants.kI, Constants.SnapConstants.kD);
        
        rotationController.enableContinuousInput(0, 2 * Math.PI);
        snapController.enableContinuousInput(0, 2 * Math.PI);

        mDriveController = new HolonomicDriveController(forwardController, strafeController, rotationController);

        SmartDashboard.putNumber("Desired traj speed", 0.0);
    }

    public double calculateRotationalAdjustment(double target_heading, double current_heading) {
        if (snapController.getSetpoint() != target_heading) {
            snapController.reset();
            snapController.setSetpoint(target_heading);
        }
        return snapController.calculate(current_heading, target_heading);
    }

    public void setTrajectory(Trajectory trajectory, Rotation2d heading, Pose2d current_pose) {
        forwardController.reset();
        strafeController.reset();
        rotationController.reset(current_pose.getRotation().getRadians());
        mStartTime = Double.NaN;
        mCurrentTrajectory = trajectory;
        isFinished = false;
        setTargetHeading(heading);
    }

    public void setTargetHeading(Rotation2d newHeading) {
        mTargetRotation = newHeading;
        if (Robot.flip_trajectories) {
            mTargetRotation = Rotation2d.fromDegrees(180.0).rotateBy(mTargetRotation.unaryMinus());
        }
    }

    public Trajectory generateTrajectory(TrajectoryConfig config, Pose2d... poses) {
        ArrayList<Translation2d> interiorPoints = new ArrayList<Translation2d>();
        for (int i = 1; i < poses.length - 1; i++) {
            interiorPoints.add(poses[i].getTranslation());
        }
        return TrajectoryGenerator.generateTrajectory(poses[0], interiorPoints, poses[poses.length - 1], config);
    }

    public ChassisSpeeds update(Pose2d current_state, double timestamp) {
        if (mStartTime.isNaN()) {
            mStartTime = Timer.getFPGATimestamp();
        }

        if (timestamp > mStartTime + mCurrentTrajectory.getTotalTimeSeconds()) {
            isFinished = true;    
        }

        if (mCurrentTrajectory == null) {
            return new ChassisSpeeds();
        }
        
        Trajectory.State desired_state = mCurrentTrajectory.sample(timestamp - mStartTime);

        SmartDashboard.putNumber("Desired traj speed", desired_state.velocityMetersPerSecond);

        return mDriveController.calculate(current_state, desired_state, mTargetRotation);
    }

    public Double getXError(double currentX, double timestamp) {
        if (mCurrentTrajectory == null) {
            return Double.NaN;
        }
        return mCurrentTrajectory.sample(timestamp - mStartTime).poseMeters.getX() - currentX;
    }

    public Double getYError(double getY, double timestamp) {
        if (mCurrentTrajectory == null) {
            return Double.NaN;
        }
        return mCurrentTrajectory.sample(timestamp - mStartTime).poseMeters.getY() - getY;
    }

    public Double getRotationalError(double currentRotation) {
        if (mTargetRotation == null) {
            return Double.NaN;
        }
        return mTargetRotation.getDegrees() - currentRotation;
    }

    public Double getRotationalTarget() {
        if (mTargetRotation == null) {
            return Double.NaN;
        }
        return mTargetRotation.getDegrees();
    }

    public boolean isFinished() {
        return mCurrentTrajectory != null && isFinished;
    }

}
