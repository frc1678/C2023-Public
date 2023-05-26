package com.team1678.frc2023.auto.actions;

import com.team1678.frc2023.subsystems.Drive;
import com.team1678.frc2023.subsystems.EndEffector;

public class WaitForAquisition implements Action {
    EndEffector effector;

    public WaitForAquisition() {
        effector = EndEffector.getInstance();
    }

    @Override
    public boolean isFinished() {
        return effector.hasGamePiece();
    }

    @Override
    public void start() {
    }

    @Override
    public void update() {

    }

    @Override
    public void done() {

    }

}
