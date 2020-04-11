package UserExamples;

import COMSETsystem.*;

import java.util.LinkedList;
import java.util.Random;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * Random destination search algorithm:
 * After dropping off a resource, the agent chooses a random intersection on the map as the destination,
 * and follows the shortest travel time path to go to the destination. When the destination is reached,
 * the agent chooses another random intersection to go to. This procedure is repeated until the agent
 * is assigned to a resource.
 */
public class AgentHubsDestination extends BaseAgent {

	// search route stored as a list of intersections.
	LinkedList<Intersection> route = new LinkedList<Intersection>();

	// random number generator
	Random rnd;

	// a static singleton object of a data model, shared by all agents
	static DummyDataModel dataModel = null;

	/**
	 * AgentRandomWalk constructor.
	 *
	 * @param id An id that is unique among all agents and resources
	 * @param map The map
	 */
	public AgentHubsDestination(long id, CityMap map) {
		super(id, map);
		rnd = new Random(id);
		if (dataModel == null) {
			dataModel = new DummyDataModel(map);
		}
	}

	/**
	 * Choose a random intersection of the map as the destination and set the
	 * shortest travel time path as the search route.
	 *
	 * IMPORTANT: The first intersection on the resulted search route must not be the
	 * end intersection of the current road, i.e., it must not be that
	 * route.get(0) == currentLocation.road.to.
	 */

	@Override
	public void planSearchRoute(LinkedList<Intersection> path) {
		route.clear();
		route = path;
		route.poll();
	}

	/**
	 * This method polls the first intersection in the current route and returns this intersection.
	 *
	 * This method is a callback method which is called when the agent reaches an intersection. The Simulator
	 * will move the agent to the returned intersection and then call this method again, and so on.
	 * This is how a planned route (in this case randomly planned) is executed by the Simulator.
	 *
	 * @return Intersection that the Agent is going to move to.
	 */

	@Override
	public Intersection nextIntersection(LocationOnRoad currentLocation, long currentTime) {
		if (route.size() != 0) {
			// Route is not empty, take the next intersection.
			Intersection nextIntersection = route.poll();
			return nextIntersection;
		} else {
			// Finished the planned route.
			return null;
		}
	}

	/**
	 * A dummy implementation of the assignedTo callback function which does nothing but clearing the current route.
	 * assignedTo is called when the agent is assigned to a resource.
	 */

	@Override
	public void clearRoute() {
		route.clear();
	}

	@Override
	public void assignedTo(LocationOnRoad currentLocation, long currentTime, long resourceId, LocationOnRoad resourcePikcupLocation, LocationOnRoad resourceDropoffLocation) {
		// Clear the current route.
		route.clear();

		Logger.getLogger(this.getClass().getName()).log(Level.INFO, "Agent " + this.id + " assigned to resource " + resourceId);
		Logger.getLogger(this.getClass().getName()).log(Level.INFO, "currentLocation = " + currentLocation);
		Logger.getLogger(this.getClass().getName()).log(Level.INFO, "currentTime = " + currentTime);
		Logger.getLogger(this.getClass().getName()).log(Level.INFO, "resourcePickupLocation = " + resourcePikcupLocation);
		Logger.getLogger(this.getClass().getName()).log(Level.INFO, "resourceDropoffLocation = " + resourceDropoffLocation);
	}

}