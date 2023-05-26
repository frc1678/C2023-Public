package com.team1678.lib.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ISwerveModule {

    public double angleOffset();

    public void setDesiredState(ModuleState desiredState, boolean isOpenLoop);
    
    public ModuleState getState();

    public Rotation2d getCanCoder();

    public void resetToAbsolute();

    public int moduleNumber();

    public double moduleOffset();
}