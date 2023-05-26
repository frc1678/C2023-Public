package com.team1678.lib.requests;

import java.util.ArrayList;
import java.util.List;

/**
 * A request type that runs all passed in requests at the same time.
 */
public class ParallelRequest extends Request {

    List<Request> requests = new ArrayList<>();
    
    /**
     * Adds any length of requests to the list of requests using variable args.
     * @param requests Can be any amount of requests from 0 - inf. Requests can be passed in with a separation of commas.
     */
    public ParallelRequest(Request... reqs) {
        for (Request request : reqs) {
            this.requests.add(request);
        }
    }

    /**
     * Appends a list of requests to the empty parallel request list.
     * @param requests List of requests.
     */
    public ParallelRequest(List<Request> reqs) {
        for (Request request : reqs) {
            this.requests.add(request);
        }
    }

    @Override
    public void act() {
        for (Request request : requests) {
            request.act();
        }        
    }

    @Override
    public boolean isFinished() {
        requests.removeIf((r) -> r.isFinished());
        return requests.isEmpty();  
    }
    
}
