package com.team1678.frc2023;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class FieldLayout {

    /**
     * Origin is the bottom left corner of the field image (Close right corner from
     * blue driver station POV)
     */

    public static final double kFieldLength = Units.inchesToMeters(651.25);
    public static final double kFieldWidth = Units.inchesToMeters(315.5);
    public static final double kTapeWidth = Units.inchesToMeters(2.0);

    // AprilTag locations
    public static final Map<Integer, Pose3d> aprilTags = Map.of(
            1, // Red Left Grid
            new Pose3d(
                    Units.inchesToMeters(610.77),
                    Units.inchesToMeters(42.19),
                    Units.inchesToMeters(18.22),
                    new Rotation3d(0.0, 0.0, Math.PI)),
            2, // Red Middle Grid
            new Pose3d(
                    Units.inchesToMeters(610.77),
                    Units.inchesToMeters(108.19),
                    Units.inchesToMeters(18.22),
                    new Rotation3d(0.0, 0.0, Math.PI)),
            3, // Red Right Grid
            new Pose3d(
                    Units.inchesToMeters(610.77),
                    Units.inchesToMeters(174.19),
                    Units.inchesToMeters(18.22),
                    new Rotation3d(0.0, 0.0, Math.PI)),
            4, // Blue Loading Station (on red alliance wall)
            new Pose3d(
                    Units.inchesToMeters(636.96),
                    Units.inchesToMeters(265.74),
                    Units.inchesToMeters(27.38),
                    new Rotation3d(0.0, 0.0, Math.PI)),
            5, // Red Loading Station (on blue alliance wall)
            new Pose3d(
                    Units.inchesToMeters(14.25),
                    Units.inchesToMeters(265.74),
                    Units.inchesToMeters(27.38),
                    new Rotation3d()),
            6, // Blue Left Grid
            new Pose3d(
                    Units.inchesToMeters(40.45),
                    Units.inchesToMeters(174.19),
                    Units.inchesToMeters(18.22),
                    new Rotation3d()),
            7, // Blue Middle Grid
            new Pose3d(
                    Units.inchesToMeters(40.45),
                    Units.inchesToMeters(108.19),
                    Units.inchesToMeters(18.22),
                    new Rotation3d()),
            8, // Blue Right Grid
            new Pose3d(
                    Units.inchesToMeters(40.45),
                    Units.inchesToMeters(42.19),
                    Units.inchesToMeters(18.22),
                    new Rotation3d()));
}
