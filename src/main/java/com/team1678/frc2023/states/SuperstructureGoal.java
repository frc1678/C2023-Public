package com.team1678.frc2023.states;

public class SuperstructureGoal {

    public static final SuperstructureGoal STOW = new SuperstructureGoal(-9.3, 0.0, 149.5);

    public static final SuperstructureGoal GROUND_CONE_INTAKE = new SuperstructureGoal(-9.3, 0.0, 21.0);
    public static final SuperstructureGoal GROUND_CUBE_INTAKE = new SuperstructureGoal(-9.3, 0.0, 21.0);

    public static final SuperstructureGoal GROUND_INTAKE_FLOAT = new SuperstructureGoal(-9.3, 0.0, 26.0);

    public static final SuperstructureGoal SHELF_CONE_INTAKE = new SuperstructureGoal(60.113914, 0.579607, -50.0);
    public static final SuperstructureGoal SHELF_CUBE_INTAKE = new SuperstructureGoal(60.113914, 0.579607, -50.0);

    public static final SuperstructureGoal YOSHI = new SuperstructureGoal(-4.101906, 0.858025, 13.707724); // 1.026

    public static final SuperstructureGoal SLIDE_INTAKE = new SuperstructureGoal(50.0, 0.00, -6.6);

    public static final SuperstructureGoal SCORE_STANDBY = new SuperstructureGoal(39.8, 0.0, 149.5);
    public static final SuperstructureGoal GROUND_SCORE = new SuperstructureGoal(15.0, 0.0, 15.0);
    public static final SuperstructureGoal L2_SCORE = new SuperstructureGoal(39.8, 0.556, -23.2);
    public static final SuperstructureGoal L2_DUNK = new SuperstructureGoal(39.8, 0.556, -20.2);
    public static final SuperstructureGoal L3_SCORE = new SuperstructureGoal(39.8, 1.04, -17.2);
    public static final SuperstructureGoal L3_DUNK = new SuperstructureGoal(39.8, 1.04, -20.2);

    public static final SuperstructureGoal CLIMB_FLOAT = new SuperstructureGoal(100.0, 0.14, 149.5);
    public static final SuperstructureGoal CLIMB_SCRAPE = new SuperstructureGoal(113.0, 0.14, 149.5);
    public static final SuperstructureGoal CLIMB_CURL = new SuperstructureGoal(-9.3, 0.14, 145.0);;

    public double arm; // degrees
    public double elevator; // meters
    public double wrist; // degrees

    public SuperstructureGoal(double arm, double elevator, double wrist) {
        this.arm = arm;
        this.elevator = elevator;
        this.wrist = wrist;
    }

    // Default Constructor
    public SuperstructureGoal() {
        this(0, 0, 0);
    }

}
