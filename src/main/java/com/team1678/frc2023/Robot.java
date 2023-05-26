// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1678.frc2023;

import java.io.IOException;
import java.net.NetworkInterface;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Enumeration;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.team1678.frc2023.auto.AutoModeExecutor;
import com.team1678.frc2023.auto.AutoModeSelector;
import com.team1678.frc2023.auto.AutoModeBase;
import com.team1678.frc2023.controlboard.ControlBoard;
import com.team1678.frc2023.controlboard.CustomXboxController.Button;
import com.team1678.frc2023.controlboard.CustomXboxController.Side;
import com.team1678.frc2023.loops.CrashTracker;
import com.team1678.frc2023.loops.Looper;
import com.team1678.frc2023.shuffleboard.ShuffleBoardInteractions;
import com.team1678.frc2023.subsystems.Arm;
import com.team1678.frc2023.subsystems.Drive;
import com.team1678.frc2023.subsystems.Elevator;
import com.team1678.frc2023.subsystems.EndEffector;
import com.team1678.frc2023.subsystems.LEDs;
import com.team1678.frc2023.subsystems.Superstructure;
import com.team1678.frc2023.subsystems.Wrist;
import com.team1678.lib.TimedRobot;
import com.team1678.lib.logger.LoggingSystem;
import com.team1678.lib.sim.PhysicsSim;
import com.team1678.lib.swerve.ChassisSpeeds;

public class Robot extends TimedRobot {

	// util instances
	private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();
	private final ControlBoard mControlBoard = ControlBoard.getInstance();
	private final ShuffleBoardInteractions mShuffleboard = ShuffleBoardInteractions.getInstance();
	private final LoggingSystem mLogger = LoggingSystem.getInstance();

	// subsystem instances
	private final Superstructure mSuperstructure = Superstructure.getInstance();
	private final Drive mDrive = Drive.getInstance();
	private final Arm mArm = Arm.getInstance();
	private final Elevator mElevator = Elevator.getInstance();
	private final Wrist mWrist = Wrist.getInstance();
	private final EndEffector mEffector = EndEffector.getInstance();
	private final LEDs mLeds = LEDs.getInstance();

	// instantiate enabled and disabled loopers
	private final Looper mEnabledLooper = new Looper();
	private final Looper mDisabledLooper = new Looper();
	private final Looper mLoggingLooper = new Looper(0.002);

	// auto instances
	private AutoModeExecutor mAutoModeExecutor;
	public final static AutoModeSelector mAutoModeSelector = new AutoModeSelector();

	public static boolean is_red_alliance = false;
	public static boolean flip_trajectories = false;

	public Robot() {
		CrashTracker.logRobotConstruction();
	}

	/* Credit to Team 2910 for this MAC Address based robot switching */
	static {
		List<byte[]> macAddresses;
		try {
			macAddresses = getMacAddresses();
		} catch (IOException e) {
			System.out.println("Mac Address attempt unsuccessful");
			System.out.println(e);
			macAddresses = List.of();
		}

		for (byte[] macAddress : macAddresses) {
			// first check if we are comp
			if (Arrays.compare(Constants.MacAddressConstants.COMP_ADDRESS, macAddress) == 0) {
				Constants.isComp = true;
				break;
			}
			// next check if we are beta
			else if (Arrays.compare(Constants.MacAddressConstants.BETA_ADDRESS, macAddress) == 0) {
				Constants.isBeta = true;
				break;
			}
			// if neither is true
			else {
				Constants.isComp = false;
				Constants.isBeta = false;
				System.out.println("New Mac Address Discovered!");
			}
		}

		if (!Constants.isComp && !Constants.isBeta) {
			// array
			String[] macAddressStrings = macAddresses.stream()
					.map(Robot::macToString)
					.toArray(String[]::new);

			SmartDashboard.putStringArray("MAC Addresses", macAddressStrings);
			// adds MAC addresses to the dashboard
			SmartDashboard.putString("Comp MAC Address", macToString(Constants.MacAddressConstants.COMP_ADDRESS));
			SmartDashboard.putString("Beta MAC Address", macToString(Constants.MacAddressConstants.BETA_ADDRESS));

			// if mac address doesn't work at comp
			Constants.isComp = true;
		}

		SmartDashboard.putBoolean("Comp Bot", Constants.isComp);
		SmartDashboard.putBoolean("Beta Bot", Constants.isBeta);

	}

	private static List<byte[]> getMacAddresses() throws IOException {
		List<byte[]> macAddresses = new ArrayList<>();

		Enumeration<NetworkInterface> networkInterfaces = NetworkInterface.getNetworkInterfaces();
		// connect to network
		NetworkInterface networkInterface;
		while (networkInterfaces.hasMoreElements()) {
			networkInterface = networkInterfaces.nextElement();

			byte[] address = networkInterface.getHardwareAddress();
			if (address == null) {
				continue;
			}

			macAddresses.add(address);
		}
		return macAddresses;
	}


	private static String macToString(byte[] address) {
		// changes string characters
		StringBuilder builder = new StringBuilder();
		for (int i = 0; i < address.length; i++) {
			if (i != 0) {
				builder.append(':');
			}
			builder.append(String.format("%02X", address[i]));
		}
		return builder.toString();
	}

	@Override
	public void robotInit() {
		try {
			CrashTracker.logRobotInit();

			LiveWindow.disableAllTelemetry();

			mSubsystemManager.setSubsystems(
					mDrive,
					mSuperstructure,
					mArm,
					mElevator,
					mWrist,
					mEffector,
					mLeds

			);

			mSubsystemManager.registerEnabledLoops(mEnabledLooper);
			mSubsystemManager.registerDisabledLoops(mDisabledLooper);

			mLoggingLooper.register(mLogger.Loop());

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void robotPeriodic() {
		mShuffleboard.update();
		mSubsystemManager.outputToSmartDashboard();
		mEnabledLooper.outputToSmartDashboard();
	}

	@Override
	public void autonomousInit() {


		try {
			mDisabledLooper.stop();
			Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode();
			if (autoMode.isPresent()) {
				mDrive.resetOdometry(autoMode.get().getStartingPose());
			}

			mEnabledLooper.start();
			mAutoModeExecutor.start();
			mLoggingLooper.start();
			
			mDrive.setNeutralBrake(true);

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
		CrashTracker.logAutoInit();

	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void autonomousExit() {
		mDrive.orientModules(List.of(
				Rotation2d.fromDegrees(45),
				Rotation2d.fromDegrees(-45),
				Rotation2d.fromDegrees(-45),
				Rotation2d.fromDegrees(45)
		));
	}
	@Override
	public void teleopInit() {
		try {
			if (is_red_alliance) {
				mDrive.zeroGyro(mDrive.getHeading().getDegrees() + 180.0);
				flip_trajectories = false;
			}
			mDisabledLooper.stop();
			mEnabledLooper.start();
			mLoggingLooper.start();
			mSuperstructure.stop();

			mDrive.setNeutralBrake(true);

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void teleopPeriodic() {
		try {

			/* Drive */
			if (mControlBoard.zeroGyro()) {
				mDrive.zeroGyro();
				mDrive.resetModulesToAbsolute();
			}

			if (mControlBoard.driver.getController().getAButtonPressed()) {
				System.out.println("Autobalance Started");
				mSuperstructure.autoBalance();
			} else if (mControlBoard.driver.getButton(Button.Y)) {
				mDrive.orientModules(List.of(
						Rotation2d.fromDegrees(45),
						Rotation2d.fromDegrees(-45),
						Rotation2d.fromDegrees(-45),
						Rotation2d.fromDegrees(45)));
			} else if (!mControlBoard.driver.getController().getAButton()) {
				mDrive.feedTeleopSetpoint(ChassisSpeeds.fromFieldRelativeSpeeds(
						mControlBoard.getSwerveTranslation().x(),
						mControlBoard.getSwerveTranslation().y(),
						mControlBoard.getSwerveRotation(),
						mDrive.getHeading()));
			}

			if (mControlBoard.driver.getController().getXButton()) {
				mDrive.setHeadingControlTarget(270.0);
			} else if (mControlBoard.driver.getController().getBButton()) {
				mDrive.setHeadingControlTarget(90.0);
			}

			/* SUPERSTRUCTURE */

			if (mControlBoard.driver.getTrigger(Side.RIGHT)) {
				mSuperstructure.setEndEffectorForwards();
			} else if ((mControlBoard.driver.getTrigger(Side.LEFT))) {
				mSuperstructure.setEndEffectorReverse();
			} else {
				mSuperstructure.setEndEffectorIdle();
			}

			if (mControlBoard.driver.getController().getPOV() == 0) {
				mWrist.setWantJog(1.0);
			} else if (mControlBoard.driver.getController().getPOV() == 180) {
				mWrist.setWantJog(-1.0);
			}

			if (mControlBoard.driver.getController().getLeftBumper()) {
				mSuperstructure.stowState();
				mDrive.setKinematicLimits(Constants.SwerveConstants.kUncappedLimits);
			} else if (mControlBoard.driver.getController().getRightBumper()) {
				mSuperstructure.chooseScoreState();
			} else if (mControlBoard.operator.getController().getRightBumper()) {
				mSuperstructure.groundIntakeState();
			} else if (mControlBoard.operator.getButton(Button.A)) {
				mSuperstructure.chooseShelfIntake();
			} else if (mControlBoard.operator.getButton(Button.Y)) {
				mSuperstructure.slideIntakeState();
			}else if (mControlBoard.operator.getButton(Button.X)) {
				mSuperstructure.scoreStandbyState();
			} else if (mControlBoard.operator.getButton(Button.B)) {
				mSuperstructure.yoshiState();
			} else if (mControlBoard.operator.getController().getLeftStickButtonPressed()) {
				mSuperstructure.climbFloatState();
			} else if (mControlBoard.operator.getController().getRightStickButtonPressed()) {
				mSuperstructure.climbScrapeState();
			} else if (mControlBoard.operator.getTrigger(Side.LEFT)) {
				mSuperstructure.climbCurlState();
			}

			if (mControlBoard.driver.getController().getLeftStickButton()) {
				mDrive.setKinematicLimits(Constants.SwerveConstants.kScoringLimits);
			} else if (mControlBoard.driver.getController().getLeftStickButtonReleased()) {
				mDrive.setKinematicLimits(Constants.SwerveConstants.kUncappedLimits);
			}

			if (mControlBoard.driver.getController().getRightStickButton()) {
				mDrive.setKinematicLimits(Constants.SwerveConstants.kLoadingStationLimits);
			} else if (mControlBoard.driver.getController().getRightStickButtonReleased()) {
				mDrive.setKinematicLimits(Constants.SwerveConstants.kUncappedLimits);
			}

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void disabledInit() {
		try {

			CrashTracker.logDisabledInit();
			mEnabledLooper.stop();
			mLoggingLooper.stop();
			mDisabledLooper.start();
			

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}

		if (mAutoModeExecutor != null) {
			mAutoModeExecutor.stop();
		}

		// Reset all auto mode state.
		mAutoModeSelector.reset();
		mAutoModeSelector.updateModeCreator(false);
		mAutoModeExecutor = new AutoModeExecutor();

	}

	@Override
	public void disabledPeriodic() {
		try {

			mDrive.resetModulesToAbsolute();

			boolean alliance_changed = false;
			if (DriverStation.isDSAttached()) {
				if (edu.wpi.first.wpilibj.DriverStation.getAlliance() == Alliance.Red) {
					if (!is_red_alliance) {
						alliance_changed = true;
					} else {
						alliance_changed = false;
					}
					is_red_alliance = true;
				} else if (edu.wpi.first.wpilibj.DriverStation.getAlliance() == Alliance.Blue) {
					if (is_red_alliance) {
						alliance_changed = true;
					} else {
						alliance_changed = false;
					}
					is_red_alliance = false;
				}
			} else {
				alliance_changed = true;
			}
			flip_trajectories = is_red_alliance;

			mAutoModeSelector.updateModeCreator(alliance_changed);
			Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode();
			if (autoMode.isPresent()) {
				mAutoModeExecutor.setAutoMode(autoMode.get());
			}

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void testInit() {
		try {
			mDisabledLooper.stop();
			mEnabledLooper.stop();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void testPeriodic() {
	}


	@Override
	public void simulationPeriodic() {
		PhysicsSim.getInstance().run();
	}
}