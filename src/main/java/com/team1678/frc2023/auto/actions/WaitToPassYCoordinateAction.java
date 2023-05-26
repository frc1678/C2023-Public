package com.team1678.frc2023.auto.actions;

import com.team1678.frc2023.subsystems.Drive;

public class WaitToPassYCoordinateAction implements Action {
	double startingXCoordinate;
	double targetXCoordinate;
	Drive drive;

	public WaitToPassYCoordinateAction(double y) {
		targetXCoordinate = y;
		drive = Drive.getInstance();
	}

	@Override
	public boolean isFinished() {
		return Math.signum(startingXCoordinate - targetXCoordinate) != Math
				.signum(drive.getPose().getTranslation().getY() - targetXCoordinate);
	}

	@Override
	public void start() {
		startingXCoordinate = drive.getPose().getTranslation().getY();
	}

	@Override
	public void update() {

	}

	@Override
	public void done() {

	}

}
