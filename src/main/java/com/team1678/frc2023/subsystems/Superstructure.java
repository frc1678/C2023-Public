package com.team1678.frc2023.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.team1678.frc2023.Constants;
import com.team1678.frc2023.controlboard.ControlBoard;
import com.team1678.frc2023.controlboard.ControlBoard.ScoringLocation;
import com.team1678.frc2023.loops.ILooper;
import com.team1678.frc2023.loops.Loop;
import com.team1678.frc2023.states.SuperstructureGoal;
import com.team1678.frc2023.subsystems.LEDs.AnimationState;
import com.team1678.frc2023.subsystems.LEDs.State;
import com.team1678.lib.logger.Log;
import com.team1678.lib.requests.ParallelRequest;
import com.team1678.lib.requests.Request;
import com.team1678.lib.requests.SequentialRequest;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Superstructure extends Subsystem {

    private Arm mArm = Arm.getInstance();
    private Elevator mElevator = Elevator.getInstance();
    private Wrist mWrist = Wrist.getInstance();
    private EndEffector mEndEffector = EndEffector.getInstance();
    private LEDs mLEDs = LEDs.getInstance();
    private Drive mDrive = Drive.getInstance();

    private Request activeRequest = null;
    private ArrayList<Request> queuedRequests = new ArrayList<>(0);
    private boolean hasNewRequest = false;
    private boolean allRequestsComplete = false;

    private boolean is_climbing = false;
    
    public boolean requestsCompleted() {
        return allRequestsComplete;
    }

    private ScoringLocation target_location = new ScoringLocation();
    private boolean wantsCube = true;

    /* Singleton Instance */
    private static Superstructure mInstance;
    public synchronized static Superstructure getInstance() {
        if (mInstance == null) {
            mInstance = new Superstructure();
        }

        return mInstance;
    };

    public void request(Request r) {
        setActiveRequest(r);
        clearRequestQueue();
    }

    private void setActiveRequest(Request request) {
        activeRequest = request;
        hasNewRequest = true;
        allRequestsComplete = false;
    }

    private void clearRequestQueue() {
        queuedRequests.clear();
    }

    private void setRequestQueue(List<Request> requests) {
        clearRequestQueue();
        for (Request req : requests) {
            queuedRequests.add(req);
        }
    }

    private void setRequestQueue(Request activeRequest, ArrayList<Request> requests) {
        request(activeRequest);
        setRequestQueue(requests);
    }

    private void addRequestToQueue(Request req) {
        queuedRequests.add(req);
    }   

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                clearRequestQueue();
                // stowState();
            }

            @Override
            public void onLoop(double timestamp) {
                try {
                    if(hasNewRequest && activeRequest != null) {
                        activeRequest.act();
                        hasNewRequest = false;
                    }
    
                    if (activeRequest == null) {
                        if (queuedRequests.isEmpty()) {
                            allRequestsComplete = true;
                        } else {
                            request(queuedRequests.remove(0));
                        }
                    } else if (activeRequest.isFinished()) {
                        activeRequest = null;
                    }
                }  catch (Exception e) {
                    e.printStackTrace();
                }

                SmartDashboard.putBoolean("All reqs complete", allRequestsComplete);

                ScoringLocation other = ControlBoard.getInstance().updateScoringLocation();
                if (!other.equals(target_location)) {
                    target_location = other;
                    System.out.println("New scoring location!");
                }
                wantsCube = (target_location.column == 2);
                SmartDashboard.putBoolean("Wants cube", wantsCube);

                if (!is_climbing) {
                    updateLEDs();
                }
            }

            @Override
            public void onStop(double timestamp) {
                clearRequestQueue();
            }
        });
    }

    public ScoringLocation getScoringLocation() {
        return target_location;
    }

    public void setScoringLocation(ScoringLocation scoringLocation) {
        target_location = scoringLocation;
    }

    @Override
    public void stop() {
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    Timer flash_timeout = new Timer();
    public void updateLEDs() {
        if (mLEDs.getUsingSmartdash()) {
            return;
        }
        State state = State.OFF;

        if (mEndEffector.hasGamePiece()) {
            if (flash_timeout.get() > 1.5) {
                if (target_location.level == 1) {
                    state = State.SOLID_BLUE;
                } else if (target_location.level == 2) {
                    state = State.SOLID_PINK;
                } else {
                    state = State.SOLID_GREEN; 
                }
                flash_timeout.stop();
            } else {
                state = State.FLASHING_GREEN;
                flash_timeout.start();
            }
        } else {
            flash_timeout.reset();
            if (wantsCube) {
                state = State.SOLID_PURPLE;
            } else {
                state = LEDs.State.SOLID_YELLOW;
            }
        } 

        mLEDs.applyStates(state);

    }

    public void setEndEffectorIdle() {
        mEndEffector.setState(EndEffector.State.IDLE); 
    }

    public void setEndEffectorForwards() {
        if (wantsCube) {
            if (target_location.level == 1) {
                mEndEffector.setState(EndEffector.State.LOW_CUBE_SPIT);   
            } else {
                mEndEffector.setState(EndEffector.State.OUTTAKING_CUBE);
            }
        } else {
            mEndEffector.setState(EndEffector.State.INTAKING_CONE);
        }
    }

    public void setEndEffectorReverse() {
        if (wantsCube) {
            mEndEffector.setState(EndEffector.State.INTAKING_CUBE);
        } else {
            if (target_location.level == 1) {
                mEndEffector.setState(EndEffector.State.LOW_CONE_SPIT);  
            } else {
                mEndEffector.setState(EndEffector.State.OUTTAKING_CONE);  
            }
        }
    }

    public void chooseShelfIntake() {
        if (wantsCube) {
            shelfCubeIntake();
        } else {
            shelfConeIntake();
        }
    }

    public void shelfConeIntake() {
        updateClimbStatus(false);
        SuperstructureGoal state = SuperstructureGoal.SHELF_CONE_INTAKE;
        request(new SequentialRequest(
                mArm.armRequest(state.arm, true),
                mElevator.elevatorRequest(state.elevator, false),
                mWrist.wristRequest(state.wrist, false)
        ));
    }

    public void shelfCubeIntake() {
        updateClimbStatus(false);
        SuperstructureGoal state = SuperstructureGoal.SHELF_CUBE_INTAKE;
        request(new SequentialRequest(
                mArm.armRequest(state.arm, true),
                mElevator.elevatorRequest(state.elevator, false),
                mWrist.wristRequest(state.wrist, false)));
    }
    
    public void stowElevator() {
        updateClimbStatus(false);
        SuperstructureGoal state = SuperstructureGoal.STOW;
        request(new SequentialRequest(
                mWrist.wristRequest(state.wrist, false),
                mElevator.elevatorRequest(state.elevator, false),
                mElevator.elevatorTuckWaitRequest(0.77)
        ));
    }

    public void stowWrist() {
        updateClimbStatus(false);
        SuperstructureGoal state = SuperstructureGoal.STOW;
        request(new SequentialRequest(
                mWrist.wristRequest(state.wrist, true)
        ));
    }

    public void stowState() {
        updateClimbStatus(false);
        SuperstructureGoal state = SuperstructureGoal.STOW;
        request(new SequentialRequest(
            mWrist.wristRequest(state.wrist, false),
            mElevator.elevatorRequest(state.elevator, true),
            mWrist.wristAboveAngleWait(0.0),
            mArm.armRequest(state.arm, true)
        ));
    }

    public void groundIntakeState() {
        updateClimbStatus(false);
        SuperstructureGoal state = SuperstructureGoal.GROUND_CONE_INTAKE;
        request(new SequentialRequest(
            mElevator.elevatorRequest(state.elevator, true),
            mArm.climbRequest(state.arm),
            mWrist.wristRequest(state.wrist, true)
        ));
    }
    
    public void groundIntakeFloatState() {
        updateClimbStatus(false);
        SuperstructureGoal state = SuperstructureGoal.GROUND_INTAKE_FLOAT;
        request(new SequentialRequest(
            mElevator.elevatorRequest(state.elevator, true),
            mArm.climbRequest(state.arm),
            mWrist.wristRequest(state.wrist, true)
        ));
    }

    public void slideIntakeState() {
        updateClimbStatus(false);
        SuperstructureGoal state = SuperstructureGoal.SLIDE_INTAKE;
        request(new SequentialRequest(
                mElevator.elevatorRequest(state.elevator, true),
                mArm.armRequest(state.arm, true),
                mWrist.wristRequest(state.wrist, true)
        ));
    }

    public void yoshiState() {
        updateClimbStatus(false);
        SuperstructureGoal state = SuperstructureGoal.YOSHI;
        request(new SequentialRequest(
                mArm.armRequest(state.arm, true),
                mElevator.elevatorRequest(state.elevator, false),
                mWrist.wristRequest(state.wrist, true)
        ));
    }
    

    // Used only in tele-op
    public void chooseScoreState() {
        switch (target_location.level) {
            case 1:
                scoreL1State();
                break;
            case 2:
				mDrive.setKinematicLimits(Constants.SwerveConstants.kScoringLimits);
                scoreL2State();
                break;
            case 3:
				mDrive.setKinematicLimits(Constants.SwerveConstants.kScoringLimits);
                scoreL3State();
                break;
            default:
                DriverStation.reportError("Unexpected Scoring State!", false);
                break;
        }
    }

    public void dunkState() {
        switch (target_location.level) {
            case 1:
                break;
            case 2:
                dunkL2State();
                break;
            case 3:
                dunkL3State();
                break;
            default:
                DriverStation.reportError("Unexpected Scoring State!", false);
                break;
        }
    }


    public void scoreStandbyState() {
        scoreStandbyState(false);
    }

    public void scoreStandbyState(boolean force) {
        if (!requestsCompleted() && !force) {
            return;
        }
        updateClimbStatus(false);
        SuperstructureGoal state = SuperstructureGoal.SCORE_STANDBY;
        request(new SequentialRequest(
            mArm.armRequest(state.arm, true),
            mElevator.elevatorRequest(state.elevator, true),
            mWrist.wristRequest(state.wrist, true)));
    }

    public void scoreL1State() {
        updateClimbStatus(false);
        SuperstructureGoal state = SuperstructureGoal.GROUND_SCORE;
        request(new SequentialRequest(
                mElevator.elevatorRequest(state.elevator, true),
                mWrist.wristRequest(state.wrist, false),
                mArm.armRequest(state.arm, false)));
    }

    public void scoreL2State() {
        updateClimbStatus(false);
        SuperstructureGoal state = SuperstructureGoal.L2_SCORE;
        request(new SequentialRequest(
                mArm.armRequest(state.arm, true),
                mElevator.elevatorRequest(state.elevator, false),
                mWrist.wristRequest(state.wrist, false)));
    }

    private void dunkL2State() {
        updateClimbStatus(false);
        SuperstructureGoal state = SuperstructureGoal.L2_DUNK;
        request(new SequentialRequest(
                mArm.armRequest(state.arm, false),
                mElevator.elevatorRequest(state.elevator, false),
                mWrist.wristRequest(state.wrist, true)));
    }

    public void scoreL3State() {
        updateClimbStatus(false);
        SuperstructureGoal state = SuperstructureGoal.L3_SCORE;
        request(new SequentialRequest(
            mWrist.wristRequest(60.0, false),
            mArm.armRequest(state.arm, true),
            mElevator.elevatorRequest(state.elevator, false),
            mElevator.elevatorExtendWaitRequest(0.77),
            mWrist.wristRequest(state.wrist, false)
        ));
    }
    
    private void dunkL3State() {
        updateClimbStatus(false);
        SuperstructureGoal state = SuperstructureGoal.L3_DUNK;
        request(new SequentialRequest(
                mArm.armRequest(state.arm, false),
                mElevator.elevatorRequest(state.elevator, false),
                mWrist.wristRequest(state.wrist, true)));
    }

    public void climbFloatState() {
        updateClimbStatus(true);
        SuperstructureGoal state = SuperstructureGoal.CLIMB_FLOAT;
        request(new SequentialRequest(
            mElevator.elevatorRequest(state.elevator, false),
            mWrist.wristRequest(state.wrist, false),
            mArm.armRequest(state.arm, false)
        ));
    }

    public void climbScrapeState() {
        updateClimbStatus(true);
        SuperstructureGoal state = SuperstructureGoal.CLIMB_SCRAPE;
        request(new SequentialRequest(
            mElevator.elevatorRequest(state.elevator, false),
            mWrist.wristRequest(state.wrist, false),
            mArm.scrapeRequest(state.arm),
            mLEDs.ledRequest(State.SOLID_CYAN)
        ));
    }

    public void climbCurlState() {
        updateClimbStatus(true);
        SuperstructureGoal state = SuperstructureGoal.CLIMB_CURL;
        request(new SequentialRequest(
                mLEDs.ledRequest(State.FLASHING_CYAN),
                mElevator.elevatorRequest(state.elevator, false),
                mWrist.wristRequest(state.wrist, false),
                mArm.climbRequest(state.arm),
                mLEDs.animationRequest(AnimationState.RAINBOW)
        ));
    }

    public void autoBalance() {
        request(Drive.getInstance().autoBalanceRequest());
    }

    private void updateClimbStatus(boolean climb) {
        if (is_climbing != climb) {
            is_climbing = climb;
        }
    }

    @Log
    public int requestQueueLength() {
        return queuedRequests.size();
    }

    @Log
    public int sequentialRequestStep() {
        if (activeRequest instanceof SequentialRequest) {
            return ((SequentialRequest) activeRequest).getListLength();
        } else {
            return -1;
        }
    }
    
    @Log
    public String activeRequest() {
        if (activeRequest == null) {
            return "null";
        } else if (activeRequest instanceof SequentialRequest) {
            return ((SequentialRequest) activeRequest).getActiveRequest();
        } else {
            return activeRequest.toString();
        }
    }
}