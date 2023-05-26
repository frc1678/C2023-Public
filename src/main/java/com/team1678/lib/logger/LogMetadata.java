package com.team1678.lib.logger;

import com.team1678.frc2023.Robot;
import com.team1678.frc2023.auto.AutoModeSelector.DesiredMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
// Data to be logged:
// Event Name
// Match Number
// Match Type
// Alliance
// Driver Station Location
// Current Auto Path
// Disable Time

public class LogMetadata {

	private String eventName;
	private int matchNumber;
	private MatchType matchType;
	private Alliance alliance;
	private int driverStationLocation;
    private long disableTimeUnixMillis;
    private DesiredMode autoMode;

    public LogMetadata(String eventName, int matchNumber, MatchType matchType, Alliance alliance, int driverStationLocation, DesiredMode autoMode, long disableTimeUnixMillis) {
		this.eventName = eventName;
		this.matchNumber = matchNumber;
		this.matchType = matchType;
		this.alliance = alliance;
		this.driverStationLocation = driverStationLocation;
        this.disableTimeUnixMillis = disableTimeUnixMillis;
        this.autoMode = autoMode;
    }

    public LogMetadata() {
        this(DriverStation.getEventName(), DriverStation.getMatchNumber(), DriverStation.getMatchType(), DriverStation.getAlliance(), DriverStation.getLocation(), Robot.mAutoModeSelector.getDesiredAutomode(), System.currentTimeMillis());
    }

    public String generateMetadataString() {
        return eventName + "\n" + matchNumber + "\n" + matchType.toString() + "\n" + alliance.toString() + "\n" + driverStationLocation + "\n" + disableTimeUnixMillis + "\n" + autoMode.toString() + "\n# eventName, matchNumber, matchType, alliance, driverStationLocation, disableTimeUnixMillis, autoMode";

    }

}
