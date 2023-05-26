package com.team1678.lib.requests;

import java.util.ArrayList;
import java.util.List;

public class SequentialRequest extends Request {

    List<Request> requests;
    Request currentRequest = null;

    /**
     * Adds in a list of requests into the local requests list using variable arguments.
     * @param reqs List of Request type arguments delineated by a comma. Can range from 0 - inf.
     */
    public SequentialRequest(Request... reqs) {
        this.requests = new ArrayList<>();
        for (Request req : reqs) {
            this.requests.add(req);
        }
    }

    /**
     * Adds a list of requets into the local request list using a set Java list.
     * @param reqs List of Request type arguments.
     */
    public SequentialRequest(List<Request> reqs) {
        this.requests = new ArrayList<>();
        for (Request req : reqs) {
            this.requests.add(req);
        }
    }

    @Override
    public void act() {
        currentRequest = requests.remove(0);
        currentRequest.act();        
    }

    @Override
    public boolean isFinished() {
        if (currentRequest == null) {
            if (requests.isEmpty()) {
                currentRequest = null;
                return true;
            } else {
                currentRequest = requests.remove(0);
                currentRequest.act();
            }
        }

        if (currentRequest.isFinished()) {
            if (requests.isEmpty()) {
                currentRequest = null;
                return true;
            } else {
                currentRequest = requests.remove(0);
                currentRequest.act();
            }
        }
        return false;
    }

    public int getListLength() {
        return requests.size();
    }

    public String getActiveRequest() {
        if (currentRequest == null) {
            return "null";
        } else {
            return currentRequest.toString();
        }
    }
    
}
