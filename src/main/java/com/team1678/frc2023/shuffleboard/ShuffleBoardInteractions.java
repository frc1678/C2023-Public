package com.team1678.frc2023.shuffleboard;

import java.util.ArrayList;

import com.team1678.frc2023.shuffleboard.tabs.OperatorTab;
import com.team1678.frc2023.shuffleboard.tabs.SwerveTab;

public class ShuffleBoardInteractions {

    // Trims unneccesary tabs when in competition
    public final boolean mDebug = true;

    /* ShuffleBoardInteractions Instance */
    private static ShuffleBoardInteractions mInstance; 

    public static ShuffleBoardInteractions getInstance() {
        if (mInstance == null) {
            mInstance = new ShuffleBoardInteractions();
        }
        return mInstance;
    }

    private ArrayList<ShuffleboardTabBase> mTabs = new ArrayList<ShuffleboardTabBase>();

    public FieldView mFieldView = new FieldView();
    private SwerveTab mSwerveTab = new SwerveTab();
    private OperatorTab mOperatorTab = new OperatorTab();

    // instantiate subsystems, tabs, and widgets
    public ShuffleBoardInteractions() {
        mTabs.add(mOperatorTab);
        mTabs.add(mSwerveTab);
        for(ShuffleboardTabBase tab: mTabs) {
            tab.createEntries();
        }
    }

    public void update() {
        for (ShuffleboardTabBase tab : mTabs) {
            tab.update();
        }
        mFieldView.update();
    }
}
 