package com.team1678.frc2023.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

import com.team1678.frc2023.auto.modes.*;

public class AutoModeSelector {
    public enum DesiredMode {
        DO_NOTHING, 
        TEST_PATH_AUTO,
        ONE_BALANCE,
        TWO_PICKUP_BALANCE,
        TWO_BALANCE,
        CC_ONE_PICKUP_BALANCE,
        CC_TWO_PICKUP_BALANCE,
        TWO_PICKUP,
        ONE_PIECE,
        CC_ONE_PIECE,
        MIDDLE_BALANCE,
        THREE_PIECE,
        CC_THREE_PIECE,
        TWO_MIDDLE_BALANCE,
    }

    private DesiredMode mCachedDesiredMode = DesiredMode.DO_NOTHING;

    private Optional<AutoModeBase> mAutoMode = Optional.empty();

    private static SendableChooser<DesiredMode> mModeChooser = new SendableChooser<>();

    public AutoModeSelector() {
        mModeChooser.addOption("Test Path", DesiredMode.TEST_PATH_AUTO);
        mModeChooser.setDefaultOption("Do Nothing", DesiredMode.DO_NOTHING);
        mModeChooser.setDefaultOption("One Balance", DesiredMode.ONE_BALANCE);
        mModeChooser.setDefaultOption("Two Pickup Balance", DesiredMode.TWO_PICKUP_BALANCE);
        mModeChooser.setDefaultOption("Two Balance", DesiredMode.TWO_BALANCE);
        mModeChooser.setDefaultOption("Two No Balance", DesiredMode.TWO_PICKUP);
        mModeChooser.setDefaultOption("Cable Chain One Pickup", DesiredMode.CC_ONE_PIECE);
        mModeChooser.setDefaultOption("Cable Chain One Pickup Balance", DesiredMode.CC_ONE_PICKUP_BALANCE);
        mModeChooser.setDefaultOption("Cable Chain Two Pickup Balance", DesiredMode.CC_TWO_PICKUP_BALANCE);
        mModeChooser.setDefaultOption("One No Balance", DesiredMode.ONE_PIECE);
        mModeChooser.setDefaultOption("Middle Balance", DesiredMode.MIDDLE_BALANCE);
        mModeChooser.setDefaultOption("Three Piece", DesiredMode.THREE_PIECE);
        mModeChooser.setDefaultOption("Cable Chain Three", DesiredMode.CC_THREE_PIECE);
        mModeChooser.setDefaultOption("Two Middle Balance", DesiredMode.TWO_MIDDLE_BALANCE);
        SmartDashboard.putData("Auto Mode", mModeChooser);
    }

    public void updateModeCreator(boolean force_regen) {
        DesiredMode desiredMode = mModeChooser.getSelected();
        if (desiredMode == null) {
            desiredMode = DesiredMode.DO_NOTHING;
        }
        if (mCachedDesiredMode != desiredMode || force_regen) {
            System.out.println("Auto selection changed, updating creator: desiredMode->" + desiredMode.name());
            mAutoMode = getAutoModeForParams(desiredMode);
        }
        mCachedDesiredMode = desiredMode;
    }

    private Optional<AutoModeBase> getAutoModeForParams(DesiredMode mode) {
        switch (mode) {
        case DO_NOTHING:
            return Optional.of(new DoNothingMode());
        
        case TEST_PATH_AUTO:
            return Optional.of(new TestPathMode());
    
        case ONE_BALANCE:
            return Optional.of(new OneBalance());
        
        case TWO_PICKUP_BALANCE:
            return Optional.of(new TwoPickupBalance());
        
        case CC_ONE_PICKUP_BALANCE:
            return Optional.of(new OneBalanceCC());

        case CC_TWO_PICKUP_BALANCE:
            return Optional.of(new TwoPickupBalanceCC());
    
        case TWO_PICKUP:
            return Optional.of(new TwoPickup());
            
        case TWO_BALANCE:
            return Optional.of(new TwoBalance());

        case ONE_PIECE:
            return Optional.of(new OnePiece());

        case CC_ONE_PIECE:
            return Optional.of(new OnePieceCC());
        
        case MIDDLE_BALANCE:
            return Optional.of(new MiddleBalance());

        case THREE_PIECE:
            return Optional.of(new ThreePiece());
            
        case CC_THREE_PIECE:
            return Optional.of(new ThreePieceCC());
            
        case TWO_MIDDLE_BALANCE:
            return Optional.of(new TwoMiddleBalance());
        default:
            System.out.println("ERROR: unexpected auto mode: " + mode);
            break;
        }

        System.err.println("No valid auto mode found for  " + mode);
        return Optional.empty();
    }

    public static SendableChooser<DesiredMode> getModeChooser() {
        return mModeChooser;
    }

    public DesiredMode getDesiredAutomode() {
        return mCachedDesiredMode;
    }

    public void reset() {
        mAutoMode = Optional.empty();
        mCachedDesiredMode = null;
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putString("AutoModeSelected", mCachedDesiredMode.name());
    }

    public Optional<AutoModeBase> getAutoMode() {
        if (!mAutoMode.isPresent()) {
            return Optional.empty();
        }
        return mAutoMode;
    }

    public boolean isDriveByCamera() {
        return mCachedDesiredMode == DesiredMode.DO_NOTHING;
    }
}
