package com.team1678.lib.requests;

import java.util.ArrayList;
import java.util.List;

/**
 * Base request class
 */
public abstract class Request {

    /**
     * Empty method that will be filled with all of the executable(s) for the given request.
     */
    public abstract void act();

    /**
     * Abstract method that returns if all of the request parameters have been met.
     * @return Default should be true if there is no check required for the next request to execute. Will run indefinately if this never returns true
     */
    public abstract boolean isFinished();

    /**
     * List that contains all of the passed in prerequisites for a request to run
     */
    public List<Prerequisite> prerequisites = new ArrayList<>();

    /**
     * Adds each individual subsyste prerequisite to a list of prerequisites which is default empty on each request creaetion
     * @param reqs List of prerequisites for a subsystem to meet prior to the request being executed
     */
    public void addPrerequisites(List<Prerequisite> reqs) {
        for (Prerequisite req : reqs) {
            prerequisites.add(req);
        }
    }

    /**
     * Adds a specific prerequisite to a list of prerequisites which is default empty on each request creation.
     * @param req A single prerequisite for a subsystem to meet prior to the request being executed.
     */
    public void addPrerequisite(Prerequisite req) {
        prerequisites.add(req);
    }

    /**
     * Verifies that all prerequisites have been met prior to allowing the request to execute
     * @return Default is true if no prerequisites are added or need to be evaluated for the subsystem to function.
     */
    public boolean allowed() {
        boolean reqMet = true;

        for (Prerequisite req : prerequisites) {
            reqMet &= req.met();
        }

        return reqMet;
    }

}