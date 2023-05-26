package com.team1678.frc2023.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team1678.frc2023.Constants;
import com.team1678.frc2023.Ports;
import com.team1678.frc2023.loops.ILooper;
import com.team1678.frc2023.loops.Loop;
import com.team1678.lib.logger.Log;
import com.team1678.lib.requests.Request;
import com.team254.lib.drivers.TalonFXFactory;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EndEffector extends Subsystem {
    public State mState = State.IDLE;
    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private boolean hasGamePiece = false;
    private double stateEnterTimestamp = 0.0;

    private final TalonFX mMaster;
    private final SupplyCurrentLimitConfiguration kSupplyCurrentLimit = new SupplyCurrentLimitConfiguration(true, 40, 40,
            0.02);
   
    private final StatorCurrentLimitConfiguration kDisabledStatorLimit = new StatorCurrentLimitConfiguration(false, 0, 0, 0);
    private final StatorCurrentLimitConfiguration kEnabledStatorLimit = new StatorCurrentLimitConfiguration(true, 40, 40,
            0.0);
        
    private EndEffector() {
        mMaster = TalonFXFactory.createDefaultTalon(Ports.END_EFFECTOR, "rio");
        mMaster.configSupplyCurrentLimit(kSupplyCurrentLimit , Constants.kLongCANTimeoutMs);
        mMaster.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(), Constants.kLongCANTimeoutMs);
        mMaster.configVoltageCompSaturation(Constants.ClawConstants.kMaxVoltage, Constants.kLongCANTimeoutMs);
        mMaster.enableVoltageCompensation(true);
        mMaster.setInverted(true);
        mMaster.setNeutralMode(NeutralMode.Brake);

        mMaster.config_kP(0, 1.5, Constants.kLongCANTimeoutMs);
        mMaster.config_kI(0, 0.0, Constants.kLongCANTimeoutMs);
        mMaster.config_kD(0, 0.5, Constants.kLongCANTimeoutMs);
        mMaster.config_kF(0, 0.0, Constants.kLongCANTimeoutMs);
    }
    
    public static EndEffector mInstance;
    public static EndEffector getInstance() {
        if (mInstance == null) {
            mInstance = new EndEffector();
        }
        return mInstance;
    }

    public enum State {
        IDLE(0.0), 
        INTAKING_CUBE(-12.0), 
        OUTTAKING_CUBE(12.0),
        
        INTAKING_CONE(12.0),
        OUTTAKING_CONE(-12.0),
        
        LOW_CUBE_SPIT(8.0),
        LOW_CONE_SPIT(-8.0);

        public double voltage;
        State (double voltage) {
            this.voltage = voltage;
        }
    }

    public State getState() {
        return mState;
    }

    public void setState(State state) {
        if (mState != state) {
            stateEnterTimestamp = Timer.getFPGATimestamp();
            if (state != State.IDLE) {
                hasGamePiece = false;
            }
            if (state == State.INTAKING_CONE || state == State.INTAKING_CUBE) {
                mMaster.configStatorCurrentLimit(kEnabledStatorLimit, Constants.kCANTimeoutMs);
            } else {
                mMaster.configStatorCurrentLimit(kDisabledStatorLimit, Constants.kCANTimeoutMs);
            }
        }
        mState = state;
    }

    @Override
    public void stop() {
        mPeriodicIO.demand = 0.0;
        setState(State.IDLE);
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    public static class PeriodicIO {
        // Inputs
        private double timestamp;
        private double voltage;
        private double current;
        private double position;
        private double velocity;

        // Outputs
        private double demand;
    }

    public void setOpenLoopDemand(double demand) {
        mPeriodicIO.demand = demand;
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                setState(State.IDLE);
            }

            @Override
            public void onLoop(double timestamp) {
                switch (mState) {
                    case IDLE:
                        mPeriodicIO.demand = mState.voltage; 
                        break; 
                    case INTAKING_CUBE:
                        mPeriodicIO.demand = mState.voltage;

                        if (mPeriodicIO.current > 40.0) {
                            hasGamePiece = true; 
                        } 

                        break; 
                    case INTAKING_CONE:
                        mPeriodicIO.demand = mState.voltage;

                        if (mPeriodicIO.current > 40.0) {
                            hasGamePiece = true; 
                        } 

                        break;
                    case OUTTAKING_CUBE:
                    case OUTTAKING_CONE:
                    case LOW_CONE_SPIT:
                    case LOW_CUBE_SPIT:
                        mPeriodicIO.demand = mState.voltage;
                        hasGamePiece = false;
                        break;
                }
            }

            @Override
            public void onStop(double timestamp) {
            }
        });
    }
    @Override
    public void writePeriodicOutputs() {
            mMaster.set(ControlMode.PercentOutput, mPeriodicIO.demand > Constants.ClawConstants.kMaxVoltage ? 1.0
                    : mPeriodicIO.demand / Constants.ClawConstants.kMaxVoltage);
    }

    public Request effectorRequest (State _wantedState) {
        return new Request () {
            @Override
            public void act() {
                setState(_wantedState); 
            }
            @Override
            public boolean isFinished() {
                return mPeriodicIO.demand == _wantedState.voltage;
            }
        };
    }

    public Request waitForGamePieceRequest () {
        return new Request() {
            @Override
            public void act () {

            }
            @Override
            public boolean isFinished() {
                return hasGamePiece;
            }
        };
    }

    public boolean hasGamePiece() {
        return hasGamePiece;
    }


    @Log
    public double getEndEffectorDemand(){
        return mPeriodicIO.demand;
    }
    
    @Log
    public double getEndEffectorVoltage(){
        return mPeriodicIO.voltage;
    }
    
    @Log
    public double getEndEffectorCurrent(){
        return mPeriodicIO.current;
    }

    @Log
    public double getTimestamp() {
        return mPeriodicIO.timestamp;
    }

    @Log
    public double getVelocity() {
        return mPeriodicIO.velocity;
    }
 
    @Log
    public double getMainMotorBusVolts() {
        return mMaster.getBusVoltage();
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Intake Demand", mPeriodicIO.demand);
        SmartDashboard.putNumber("Intake Volts", mPeriodicIO.voltage);
        SmartDashboard.putNumber("Intake Current", mPeriodicIO.current);
        SmartDashboard.putString("Intake State", mState.toString());
        SmartDashboard.putBoolean("Has game piece", hasGamePiece); 
    }

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();
        mPeriodicIO.voltage = mMaster.getMotorOutputVoltage();
        mPeriodicIO.current = mMaster.getStatorCurrent();
        mPeriodicIO.position = mMaster.getSelectedSensorPosition();
        mPeriodicIO.velocity = mMaster.getSelectedSensorVelocity();
    }
}
