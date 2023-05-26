package com.team1678.frc2023.shuffleboard.tabs;

import com.team1678.frc2023.shuffleboard.ShuffleboardTabBase;
import com.team1678.frc2023.subsystems.Drive;
import com.team1678.lib.swerve.SwerveModule;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public class SwerveTab extends ShuffleboardTabBase {

    private Drive mDrive = Drive.getInstance();

    private final SwerveModule[] mDriveModules;

    private String[] kSwervePlacements = {"Front Left", "Front Right", "Back Left", "Back Right"};
    private ShuffleboardLayout[] mDriveLayouts = new ShuffleboardLayout[4];
    private GenericEntry[] mDriveCancoders = new GenericEntry[4];
    private GenericEntry[] mDriveIntegrated = new GenericEntry[4];
    private GenericEntry[] mDriveMPS = new GenericEntry[4];
    private GenericEntry[] mDriveCurrent = new GenericEntry[4];

    private GenericEntry mDriveOdometryX;
    private GenericEntry mDriveOdometryY;
    private GenericEntry mDriveOdometryRot;

    public SwerveTab() {
        super();
        mDriveModules = mDrive.mModules;
    }

    @Override
    public void createEntries() {
        mTab = Shuffleboard.getTab("Swerve");

        for (int i = 0; i < mDriveCancoders.length; i++) {
            mDriveLayouts[i] = mTab
                .getLayout("Module " + i + " Angle", BuiltInLayouts.kGrid)
                .withSize(2, 2)
                .withPosition(i * 2, 0);
            mDriveCancoders[i] = mDriveLayouts[i].add("Cancoder", 0.0)
                .withPosition(0, 0)
                .withSize(5, 1)
                .getEntry();
            mDriveLayouts[i].add("Location", kSwervePlacements[i])
                .withPosition(1, 0)
                .withSize(5, 1);
            
            mDriveIntegrated[i] = mDriveLayouts[i].add("Integrated", 0.0)
                .withPosition(0, 1)
                .withSize(5, 1)
                .getEntry();
            mDriveLayouts[i].add("Offset", mDriveModules[i].angleOffset())
                .withPosition(0, 2)
                .withSize(5, 1)
                .getEntry();
            mDriveCurrent[i] = mDriveLayouts[i]
                    .add("Stator Current ", 0.0)
                    .withPosition(1, 2)
                    .withSize(5, 1)
                    .getEntry();

            mDriveMPS[i] = mTab
                .add("Swerve Module " + i + " MPS ", 0.0)
                .withPosition(i * 2, 2)
                .withSize(2, 1)
                .getEntry();
            
 
        }

        mDriveOdometryX = mTab
            .add("Odometry X", 0)
            .withPosition(0, 3)
            .withSize(2, 1)
            .getEntry();
        mDriveOdometryY = mTab
            .add("Odometry Y", 0)
            .withPosition(2, 3)
            .withSize(2, 1)
            .getEntry();
        mDriveOdometryRot = mTab
            .add("Pigeon Angle", 0)
            .withPosition(4, 3)
            .withSize(2, 1)
            .getEntry();
    }

    @Override
    public void update() {
        for (int i = 0; i < mDriveCancoders.length; i++) {
            // mDriveCancoders[i].setDouble(truncate(mDriveModules[i].getCanCoder().getDegrees()));
            mDriveIntegrated[i].setDouble(truncate(MathUtil.inputModulus(mDriveModules[i].getState().angle.getDegrees(), 0, 360)));
            mDriveMPS[i].setDouble(mDriveModules[i].getState().speedMetersPerSecond);
            mDriveCurrent[i].setDouble(mDriveModules[i].getDriveMotorCurrent());
        }
        
        mDriveOdometryX.setDouble(truncate(mDrive.getPose().getX()));
        mDriveOdometryY.setDouble(truncate(mDrive.getPose().getY()));
        mDriveOdometryRot.setDouble(truncate(MathUtil.inputModulus(mDrive.getPose().getRotation().getDegrees(), 0, 360)));

    }

}
