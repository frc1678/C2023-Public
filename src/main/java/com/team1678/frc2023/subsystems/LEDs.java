package com.team1678.frc2023.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdleControlFrame;
import com.ctre.phoenix.led.CANdleStatusFrame;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.team1678.frc2023.Constants;
import com.team1678.frc2023.Ports;
import com.team1678.frc2023.loops.ILooper;
import com.team1678.frc2023.loops.Loop;
import com.team1678.lib.requests.Request;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LEDs extends Subsystem {

    private final CANdle mCandle = new CANdle(Ports.CANDLE, "canivore1");

    private static LEDs mInstance;

    private final int kNumLeds = 59;

    public static LEDs getInstance() {
        if (mInstance == null) {
            mInstance = new LEDs();
        }
        return mInstance;
    }

    private double timestamp = 0.0;

    private final boolean mUseSmartdash = false; // if we want to manual control lights using shuffleboard

    // led sections
    private LEDStatus mLEDStatus = new LEDStatus(0, kNumLeds); 

    // Animation instances
    private Animation mRainbowAnimation = new RainbowAnimation(1.0, 1.0, kNumLeds);

    // shuffleboard selectors
    private SendableChooser<State> mStateChooser;

    // led states
    public enum State {
        OFF("OFF", Double.POSITIVE_INFINITY, Color.off()),
        EMERGENCY("EMERGENCY", 0.2, new Color(255, 0, 0), Color.off()),

        SOLID_RED("SOLID_RED", Double.POSITIVE_INFINITY, new Color(255, 0, 0)),
        SOLID_PINK("SOLID_PINK", Double.POSITIVE_INFINITY, new Color(255, 18, 143)),
        SOLID_GREEN("SOLID_GREEN", Double.POSITIVE_INFINITY, new Color(0, 255, 8)),
        SOLID_PURPLE("SOLID_PURPLE", Double.POSITIVE_INFINITY, new Color(196, 18, 255)),
        SOLID_ORANGE("SOLID_ORANGE", Double.POSITIVE_INFINITY, new Color(255, 53, 13)),
        SOLID_YELLOW("SOLID_YELLOW", Double.POSITIVE_INFINITY, new Color(255, 150, 5)),
        SOLID_CYAN("SOLID_CYAN", Double.POSITIVE_INFINITY, new Color(52, 155, 235)),
        SOLID_BLUE("SOLID_BLUE", Double.POSITIVE_INFINITY, new Color(0, 0, 255)),
        DIM_GREEN("DIM_GREEN", Double.POSITIVE_INFINITY, new Color(0, 50, 0)),

        FLASHING_RED("FLASHING_RED", 0.2, new Color(255, 0, 0), Color.off()),
        FLASHING_PINK("FLASHING_PINK", 0.2, new Color(255, 20, 0), Color.off()),
        FLASHING_GREEN("FLASHING_GREEN", 0.2, new Color(0, 255, 0), Color.off()),
        FLASHING_PURPLE("FLASHING_PURPLE", 0.2, new Color(255, 0, 255), Color.off()),
        FLASHING_ORANGE("FLASHING_ORANGE", 0.2, new Color(255, 53, 0), Color.off()),
        FLASHING_YELLOW("FLASHING_YELLOW", 0.2, new Color(255, 247, 5), Color.off()),
        FLASHING_CYAN("FLASHING_CYAN", 0.2, new Color(52, 155, 235), Color.off());

        Color[] colors; // array of colors to iterate over
        double interval; // time in seconds between states
        String name; // name of state

        private State(String name, double interval, Color... colors) {
            this.colors = colors;
            this.interval = interval;
            this.name = name;
        }

        public String getName() {
            return name;
        }
    }

    // Available animations
    public enum AnimationState {
        RAINBOW()
    }

    public LEDs() {
        configureCandle(); // set CTRE configurations for CANdle

        // create sendable choosers for shuffleboard
        if (mUseSmartdash) {
            mStateChooser = new SendableChooser<>();
            for (State state : State.values()) {
                mStateChooser.addOption(state.getName(), state); 
            }
            mStateChooser.setDefaultOption("OFF", State.OFF); 
            SmartDashboard.putData("LEDs", mStateChooser);
        }

        applyStates(State.DIM_GREEN);
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                // empty
            }

            @Override
            public void onLoop(double timestamp) {
            }

            @Override
            public void onStop(double timestamp) {
                mLEDStatus.reset(); 
                applyStates(State.DIM_GREEN);
            }
        });
    }

    @Override
    public void readPeriodicInputs() {
        updateLeds();
        outputTelemtry();
        timestamp = Timer.getFPGATimestamp(); // update timestamp for color cycling
        if (mUseSmartdash) { // pull states from smartdash
            applyStates(mStateChooser.getSelected());
        }
    }

    private void updateLeds() {
        // check if we need to cycle to next color
        if (mLEDStatus.state.interval != Double.POSITIVE_INFINITY) {
            if (timestamp - mLEDStatus.lastSwitchTime >= mLEDStatus.state.interval) {
                mLEDStatus.nextColor();
                mLEDStatus.lastSwitchTime = timestamp;
            }
        }

        Color color = mLEDStatus.getWantedColor();

        mCandle.setLEDs(color.r, color.g, color.b, 0, mLEDStatus.startIDx,
                mLEDStatus.LEDCount);

    }


    // setter functions
    public void applyStates(State state) {
        clearAnimation();
        mLEDStatus.setState(state);
    }

    public void applyAnimation(AnimationState animation) {
        switch (animation) {
            case RAINBOW:
                mCandle.animate(mRainbowAnimation);
                break;
        
            default:
                break;
        }
    }

    // apply configuration to candle
    private void configureCandle() {
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = false;
        configAll.disableWhenLOS = true;
        configAll.stripType = LEDStripType.RGB;
        configAll.brightnessScalar = 1.0;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        mCandle.configAllSettings(configAll, Constants.kLongCANTimeoutMs);
        mCandle.setStatusFramePeriod(CANdleStatusFrame.CANdleStatusFrame_Status_1_General, 255);
        mCandle.setControlFramePeriod(CANdleControlFrame.CANdle_Control_1_General, 10);
        mCandle.setControlFramePeriod(CANdleControlFrame.CANdle_Control_2_ModulatedVBatOut, 255);
    }

    public void clearAnimation() {
        mCandle.clearAnimation(0);
    }

    @Override
    public void stop() {
        // No-op
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    // getter functions
    public State getState() {
        return mLEDStatus.state;
    }

    public boolean getUsingSmartdash() {
        return mUseSmartdash;
    }

    private void outputTelemtry() {
        SmartDashboard.putString("LED Status", getState().name); 
        SmartDashboard.putString("LED Colors", mLEDStatus.getWantedColor().toString());
    }

    public Request ledRequest(State wanted_state) {
        return new Request() {

            @Override
            public void act() {
                applyStates(wanted_state);
            }

            @Override
            public boolean isFinished() {
                return true;
            }
            
        };
    }

    public Request animationRequest(AnimationState wanted_state) {
        return new Request() {

            @Override
            public void act() {
                applyAnimation(wanted_state);
            }

            @Override
            public boolean isFinished() {
                return true;
            }

        };
    }

    // class for holding information about each section
    private class LEDStatus {
        private State state = State.OFF; // current state
        private double lastSwitchTime = 0.0; // timestampe of last color cycle
        private int colorIndex = 0; // tracks current color in array
        private int startIDx, LEDCount; // start and end of section

        public LEDStatus(int startIndex, int endIndex) {
            startIDx = startIndex;
            LEDCount = endIndex - startIndex;
        }

        public void setState(State wantedState) {
            if (wantedState != state) {
                colorIndex = 0;
                lastSwitchTime = Timer.getFPGATimestamp();
                state = wantedState;
            }
        }

        public Color getWantedColor() {
            Color color;
            try {
                color = state.colors[colorIndex];
            } catch (Exception e) {
                color = Color.off();
            }
            return color;
        }

        // cycle to next color in array
        public void nextColor() {
            if (state.colors.length == 1) {
                return;
            }
            if (colorIndex == state.colors.length - 1) {
                colorIndex = 0;
            } else {
                colorIndex++;
            }
        }

        public void reset() {
            state = State.OFF;
            lastSwitchTime = 0.0;
            colorIndex = 0;
        }
    }

    // class to hold rgb values for a color
    private static class Color {
        public int r;
        public int g;
        public int b;

        public Color(int red, int green, int blue) {
            r = red;
            g = green;
            b = blue;
        }

        public static Color off() {
            return new Color(0, 0, 0);
        }

        @Override
        public String toString() {
            return "(" + r + "," + g + "," + b + ")";
        }
    }
}
