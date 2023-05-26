package com.team1678.frc2023.controlboard;

import com.team1678.frc2023.Constants;
import com.team1678.frc2023.controlboard.CustomXboxController.Axis;
import com.team1678.frc2023.controlboard.CustomXboxController.Button;
import com.team1678.frc2023.controlboard.CustomXboxController.Side;
import com.team1678.frc2023.subsystems.Drive;
import com.team1678.lib.Util;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ControlBoard {
    private final double kSwerveDeadband = Constants.stickDeadband;

    private static ControlBoard mInstance = null;

    public enum SwerveCardinal {
        NONE(0),
        FRONT(0),
        LEFT(90),
        RIGHT(-90),
        BACk(180);

        public final double degrees;

        SwerveCardinal(double degrees) {
            this.degrees = degrees;
        }
    }

    public static ControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new ControlBoard();
        }

        return mInstance;
    }

    public final CustomXboxController driver;
    public final CustomXboxController operator;

    private ControlBoard() {
        driver = new CustomXboxController(0);
        operator = new CustomXboxController(Constants.kButtonGamepadPort);
    }

    /* DRIVER METHODS */
    public Translation2d getSwerveTranslation() {
        double forwardAxis = driver.getController().getRawAxis(1);
        double strafeAxis = driver.getController().getRawAxis(0);

        SmartDashboard.putNumber("Raw Y", forwardAxis);
        SmartDashboard.putNumber("Raw X", strafeAxis);

        forwardAxis = Constants.SwerveConstants.invertYAxis ? forwardAxis : -forwardAxis;
        strafeAxis = Constants.SwerveConstants.invertXAxis ? strafeAxis : -strafeAxis;

        Translation2d tAxes = new Translation2d(forwardAxis, strafeAxis);

        if (Math.abs(tAxes.norm()) < kSwerveDeadband) {
            return new Translation2d();
        } else {
            Rotation2d deadband_direction = new Rotation2d(tAxes.x(), tAxes.y(), true);
            Translation2d deadband_vector = Translation2d.fromPolar(deadband_direction, kSwerveDeadband);

            double scaled_x = Util.scaledDeadband(forwardAxis, 1.0, Math.abs(deadband_vector.x()));
            double scaled_y = Util.scaledDeadband(strafeAxis, 1.0, Math.abs(deadband_vector.y()));
            return new Translation2d(scaled_x, scaled_y).scale(Drive.getInstance().getKinematicLimits().kMaxDriveVelocity);
        }
    }

    public double getSwerveRotation() {
        double rotAxis = driver.getAxis(Side.RIGHT, Axis.X) * 0.80;
        rotAxis = Constants.SwerveConstants.invertRAxis ? rotAxis : -rotAxis;

        if (Math.abs(rotAxis) < kSwerveDeadband) {
            return 0.0;
        } else {
            return Drive.getInstance().getKinematicLimits().kMaxAngularVelocity * (rotAxis - (Math.signum(rotAxis) * kSwerveDeadband))
                    / (1 - kSwerveDeadband);
        }
    }

    public boolean zeroGyro() {
        return driver.getButton(Button.START) && driver.getButton(Button.BACK);
    }

    public SwerveCardinal getSwerveSnap() {
        if (driver.getButton(Button.A)) {
            return SwerveCardinal.BACk;
        }
        if (driver.getButton(Button.B)) {
            return SwerveCardinal.RIGHT;
        }
        if (driver.getButton(Button.X)) {
            return SwerveCardinal.LEFT;
        }
        if (driver.getButton(Button.Y)) {
            return SwerveCardinal.FRONT;
        }
        return SwerveCardinal.NONE;
    }

    public static class ScoringLocation {
        public int grid = 2;
        public int column = 2;
        public int level = 3;

        public ScoringLocation() {
        }

        public ScoringLocation(ScoringLocation other) {
            this.grid = other.grid;
            this.column = other.column;
            this.level = other.level;
        }

        @Override
        public String toString() {
            return "(Targeting Location: Grid " + grid + " column " + column + " row " + level + ")"; 
        }

        @Override
        public boolean equals(Object obj) {
            if (obj == null || !(obj instanceof ScoringLocation)) return false;
            ScoringLocation other = (ScoringLocation) obj;
            return (other.grid == this.grid && other.column == this.column && other.level == this.level);
        }
    }

    private ScoringLocation mLastLocation = new ScoringLocation();
    private int last_dpad;

    public ScoringLocation updateScoringLocation() {
        ScoringLocation scoring_location = new ScoringLocation(mLastLocation);

        if(operator.getController().getLeftBumperPressed()){
            if(scoring_location.grid != 1){
                scoring_location.grid--;
            }  
        }

        if(operator.getController().getRightBumperPressed()){
            if(scoring_location.grid != 3){
                scoring_location.grid++;  
            }
        }

        if (operator.getDPad() != last_dpad) {
            if (operator.getDPad() == 0) {
                if (scoring_location.level != 3) {
                    scoring_location.level++;
                }
            }

            if (operator.getDPad() == 270) {
                if (scoring_location.column != 1) {
                    scoring_location.column--;
                }
            }

            if (operator.getDPad() == 90) {
                if (scoring_location.column != 3) {
                    scoring_location.column++;
                }
            }
            if (operator.getDPad() == 180) {
                if (scoring_location.level != 1) {
                    scoring_location.level--;
                }
            }
        }

        last_dpad = operator.getDPad();
        mLastLocation = scoring_location;

        return scoring_location;
    }  

    public void setScoringLocation(ScoringLocation location) {
        mLastLocation = location;
    }
}