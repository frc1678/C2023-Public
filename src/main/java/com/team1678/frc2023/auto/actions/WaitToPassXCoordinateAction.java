package com.team1678.frc2023.auto.actions;

import com.team1678.frc2023.FieldLayout;
import com.team1678.frc2023.Robot;
import com.team1678.frc2023.subsystems.Drive;

public class WaitToPassXCoordinateAction implements Action{
	double startingXCoordinate;
	double targetXCoordinate;
	Drive drive;
	
	public WaitToPassXCoordinateAction(double x){
		if (Robot.flip_trajectories) {
			targetXCoordinate = FieldLayout.kFieldLength - x;
		} else {
			targetXCoordinate = x;
		}
		drive = Drive.getInstance();
	}
	
	@Override
	public boolean isFinished() {
		return Math.signum(startingXCoordinate - targetXCoordinate) !=
				Math.signum(drive.getPose().getTranslation().getX() - targetXCoordinate);
	}

	@Override
	public void start() {
		startingXCoordinate = drive.getPose().getTranslation().getX();
	}

	@Override
	public void update() {
		
	}

	@Override
	public void done() {
		
	}

}
