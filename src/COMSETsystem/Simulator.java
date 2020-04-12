package COMSETsystem;

import MapCreation.*;

import java.text.NumberFormat;
import java.util.*;
import java.util.concurrent.TimeUnit;

import UserExamples.HungarianAlgorithm;
import me.tongfei.progressbar.*;


import DataParsing.*;


/**
 * The Simulator class defines the major steps of the simulation. It is
 * responsible for loading the map, creating the necessary number of agents,
 * creating a respective AgentEvent for each of them such that they are added
 * to the events PriorityQueue. Furthermore it is also responsible for dealing
 * with the arrival of resources, map matching them to the map, and assigning
 * them to agents. This produces the score according to the scoring rules.
 * <p>
 * The time is simulated by the events having a variable time, this time
 * corresponds to when something will be empty and thus needs some
 * interaction (triggering). There's an event corresponding to every existent
 * Agent and for every resource that hasn't arrived yet. All of this events are
 * in a PriorityQueue called events which is ordered by their time in an
 * increasing way.
 */
public class Simulator {

	// The map that everything will happen on.
	protected CityMap map;

	// A deep copy of map to be passed to agents.
	// This is a way to make map unmodifiable.
	protected CityMap mapForAgents;

	// The event queue.
	protected PriorityQueue<Event> events = new PriorityQueue<>();

	// The set of empty agents.
	protected TreeSet<AgentEvent> emptyAgents = new TreeSet<>(new AgentEventComparator());

	// The set of resources that with no agent assigned to it yet.
	protected TreeSet<ResourceEvent> waitingResources = new TreeSet<>(new ResourceEventComparator());

	// The maximum life time of a resource in seconds. This is a parameter of the simulator.
	public long ResourceMaximumLifeTime;

	// Full path to an OSM JSON map file
	protected String mapJSONFile;


	// Full path to a TLC New York Yellow trip record file
	protected String resourceFile = null;

	// Full path to a KML defining the bounding polygon to crop the map
	protected String boundingPolygonKMLFile;

	// The simulation end time is the expiration time of the last resource.
	protected long simulationEndTime;

	// Total trip time of all resources to which agents have been assigned.
	protected long totalResourceTripTime = 0;

	// Total wait time of all resources. The wait time of a resource is the amount of time
	// since the resource is introduced to the system until it is picked up by an agent.
	protected long totalResourceWaitTime = 0;

	// Total search time of all agents. The search time of an agent for a research is the amount of time
	// since the agent is labeled as empty, i.e., added to emptyAgents, until it picks up a resource.
	protected long totalAgentSearchTime = 0;

	// Total cruise time of all agents. The cruise time of an agent for a research is the amount of time
	// since the agent is labeled as empty until it is assigned to a resource.
	protected long totalAgentCruiseTime = 0;

	// Total approach time of all agents. The approach time of an agent for a research is the amount of time
	// since the agent is assigned to a resource until agent reaches the resource.
	protected long totalAgentApproachTime = 0;

	// The number of expired resources.
	protected long expiredResources = 0;

	// The number of resources that have been introduced to the system.
	protected long totalResources = 0;

	// The number of agents that are deployed (at the beginning of the simulation).
	protected long totalAgents;

	// The number of assignments that have been made.
	protected long totalAssignments = 0;

	// A list of all the agents in the system. Not really used in COMSET, but maintained for
	// a user's debugging purposes.
	ArrayList<BaseAgent> agents;

	protected long initialPoolTime;
	protected long endPooltime=0;

	protected List<AgentEvent> assignedAgents = new ArrayList<>();

	// A class that extends BaseAgent and implements a search routing strategy
	protected final Class<? extends BaseAgent> agentClass;


	//Snehal
	protected GeoProjector geoProj;
	protected long FirstResourceTime;
	protected int numberOfPools;
	/**
	 * Constructor of the class Main. This is made such that the type of
	 * agent/resourceAnalyzer used is not hardcoded and the users can choose
	 * whichever they wants.
	 *
	 * @param agentClass the agent class that is going to be used in this
	 * simulation.
	 */
	public Simulator(Class<? extends BaseAgent> agentClass) {
		this.agentClass = agentClass;
	}

	/**
	 * Configure the simulation system including:
	 *
	 * 1. Create a map from the map file and the bounding polygon KML file.
	 * 2. Load the resource data set and map match.
	 * 3. Create the event queue.
	 *
	 * See Main.java for detailed description of the parameters.
	 *
	 * @param mapJSONFile The map file
	 * @param resourceFile The dataset file
	 * @param totalAgents The total number of agents to deploy
	 * @param boundingPolygonKMLFile The KML file defining a bounding polygon of the simulated area
	 * @param maximumLifeTime The maximum life time of a resource
	 * @param agentPlacementRandomSeed The see for the random number of generator when placing the agents
	 * @param speedReduction The speed reduction to accommodate traffic jams and turn delays
	 */
	public void configure(String mapJSONFile, String resourceFile, Long totalAgents, String boundingPolygonKMLFile, Long maximumLifeTime, long agentPlacementRandomSeed, double speedReduction) {

		this.mapJSONFile = mapJSONFile;

		this.totalAgents = totalAgents;

		this.boundingPolygonKMLFile = boundingPolygonKMLFile;

		this.ResourceMaximumLifeTime = maximumLifeTime;

		this.resourceFile = resourceFile;

		MapCreator creator = new MapCreator(this.mapJSONFile, this.boundingPolygonKMLFile, speedReduction);
		System.out.println("Creating the map...");

		creator.createMap();

		// Output the map
		map = creator.outputCityMap();

		// Pre-compute shortest travel times between all pairs of intersections.
		System.out.println("Pre-computing all pair travel times...");
		map.calcTravelTimes();

		//// Pre-compute shortest distance between all pairs of intersections.
		System.out.println("Pre-computing all pair travel distance...");
		map.calcTravelDistances();

		// Make a map copy for agents to use so that an agent cannot modify the map used by
		// the simulator
		mapForAgents = map.makeCopy();

		MapWithData mapWD = new MapWithData(map, this.resourceFile, agentPlacementRandomSeed);

		// map match resources
		System.out.println("Loading and map-matching resources..." + numberOfPools);
		long latestResourceTime = mapWD.createMapWithData(this);
		initialPoolTime = mapWD.earliestResourceTime;

		// The simulation end time is the expiration time of the last resource.
		this.simulationEndTime = latestResourceTime;
		// Deploy agents at random locations of the map.
		System.out.println("Randomly placing " + this.totalAgents + " agents on the map...");
		agents = mapWD.placeAgentsRandomly(this);
		events = mapWD.getEvents();

	}

	/**
	 * My method to try and create preference lists of resources
	 */

	//Snehal
	public void createCostMatrix(){
		//assignedAgents = new ArrayList<>();
		List<List<Double>> costMatrix = new ArrayList<List<Double>>();//For cost matrix
		List<Double> resAgent = new ArrayList<>(); //Resource to agent benefits
		List<AgentEvent> agentInResList = new ArrayList<>();//List of agents for mapping
		HashMap<ResourceEvent, Map<AgentEvent, LocationOnRoad>> ResAgentLocationOnRoad= new HashMap<>();//Store resource to agent locationOnRoad

		Map<AgentEvent,LocationOnRoad> AgentOnRoad = new HashMap<>();
		Map<Integer,List> IdAgent = new HashMap<>();//Id to agent list mapping
		Map<Integer,ResourceEvent> IdResource = new HashMap<>();//Id to resource mapping

		HashMap<ResourceEvent, Map<AgentEvent, Long>> ResAgentArriveTime= new HashMap<>();//Store resource to agent locationOnRoad
		Map<AgentEvent,Long> agentArriveTime = new HashMap<>();

		int id=0;
		for(ResourceEvent event:ResourceEvent.resList){
				//events.remove(event);
				agentInResList = new ArrayList<>();//Snehal
				resAgent = new ArrayList<>();//Snehal
				AgentEvent bestAgent = null;
				AgentEvent farAgent=null;
				long farthest= Long.MIN_VALUE;
				long earliest = Long.MAX_VALUE;
				LocationOnRoad bestAgentLocationOnRoad = null;
				ArrayList<AgentEvent> aList=new ArrayList<AgentEvent>(10);
				AgentOnRoad = new HashMap<>();
				for (AgentEvent agent : emptyAgents) {
					long travelTimeToEndIntersection = agent.time - ((ResourceEvent) event).time;
					long travelTimeFromStartIntersection = agent.loc.road.travelTime - travelTimeToEndIntersection;
					LocationOnRoad agentLocationOnRoad = new LocationOnRoad(agent.loc.road, travelTimeFromStartIntersection);
					//Store the locationOnRoad of the agent in the map
					long travelTime = map.travelTimeBetween(agentLocationOnRoad, ((ResourceEvent) event).pickupLoc);
					long arriveTime = travelTime +((ResourceEvent) event).time;


					//New distance for benefits
					long distance=map.travelDistanceBetween(agentLocationOnRoad,((ResourceEvent) event).pickupLoc);
					long tripDistance=map.travelDistanceBetween(((ResourceEvent) event).pickupLoc,((ResourceEvent) event).dropoffLoc);
					double benefit= (double)tripDistance/(tripDistance + distance);

					if(arriveTime <= ((ResourceEvent) event).expirationTime)
					{
						resAgent.add(benefit);
						agentInResList.add(agent);
						AgentOnRoad.put(agent,agentLocationOnRoad);
						//Store the arrive time for each agent
						agentArriveTime.put(agent,arriveTime);
					}
					else{
						resAgent.add(0.0);
						agentInResList.add(null);
					}
				}
				if (Collections.frequency(resAgent, 0.0) == resAgent.size())
				{
					event.time = event.time + ResourceMaximumLifeTime;
					event.eventCause = event.EXPIRED;
					waitingResources.add(event);
					continue;
				}
				ResAgentLocationOnRoad.put(event, AgentOnRoad);
				ResAgentArriveTime.put(event,agentArriveTime);
				costMatrix.add(resAgent);
				IdAgent.put(id,agentInResList);//Snehal
				IdResource.put(id, (ResourceEvent) event);//Snehal
				id++;



		}
		if(costMatrix.size()!=0) {
			System.out.println("##Hungarian - Agent in resource list" + agentInResList.size());
			System.out.println("##Hungarian - resource agent " + resAgent.size());
			System.out.println("##Costmatrix - " + costMatrix.size());
			System.out.println("Final matrix size " + costMatrix.size());
			double[][] array = costMatrix.stream().map(l -> l.stream().mapToDouble(i -> i).toArray()).toArray(double[][]::new);
			int[][] assignment = HungarianAlgorithm.hgAlgorithmAssignments(array, "max");
			double cost = HungarianAlgorithm.hgAlgorithm(array, "max");
			System.out.println("cost" + cost);
			long prevResId = -1, prevAgentId = -1;
			double repCost = 0;
			int count = 0;
			int assignmentSize = Math.min(resAgent.size(), costMatrix.size());
			for (int i = 0; i < assignmentSize; i++) {

				ResourceEvent resEvent = IdResource.get(assignment[i][0]);
				List<AgentEvent> ag = IdAgent.get(assignment[i][0]);
				//AgentEvent agentEvent = ag.get(assignment[i][1]);
				if (array[assignment[i][0]][assignment[i][1]] != 0.0) {
					System.out.println("Resource Id " + resEvent.id + " assigned to " + "Agent Id " + ag.get(assignment[i][1]).id + " Benefit " + array[assignment[i][0]][assignment[i][1]]);
					waitingResources.remove(resEvent);
					ResourceEvent.resList.remove(resEvent);
					events.remove(resEvent);
					AgentEvent bestAgent = ag.get(assignment[i][1]); //Agent assigned to the resource
					// Inform the assignment to the agent.
					LocationOnRoad agentLocation = ResAgentLocationOnRoad.get(resEvent).get(bestAgent);
					bestAgent.assignedTo(agentLocation, resEvent.time, resEvent.id, resEvent.pickupLoc, resEvent.dropoffLoc);
					// "Label" the agent as occupied.
					assignedAgents.add(bestAgent);
					emptyAgents.remove(bestAgent);
					events.remove(bestAgent);
					long agentArrival = ResAgentArriveTime.get(resEvent).get(bestAgent);
					bestAgent.setEvent(agentArrival + resEvent.tripTime, resEvent.dropoffLoc, AgentEvent.DROPPING_OFF);
					events.add(bestAgent);

					long cruiseTime = resEvent.time - bestAgent.startSearchTime;
					long approachTime = agentArrival - resEvent.time;
					long searchTime = cruiseTime + approachTime;
					long waitTime = agentArrival - resEvent.availableTime;

					totalAgentCruiseTime += cruiseTime;
					totalAgentApproachTime += approachTime;
					totalAgentSearchTime += searchTime;
					totalResourceWaitTime += waitTime;
					totalAssignments++;
				}
			}
		}

	}

	/**
	 * This method corresponds to running the simulation. An object of ScoreInfo
	 * is created in order to keep track of performance in the current
	 * simulation. Go through every event until the simulation is over.
	 *
	 * @throws Exception since triggering events may create an Exception
	 */
	public void run() throws Exception {
		Event tocheck=null;
		int resources=0;
		System.out.println("Running the simulation...");
		initialPoolTime=initialPoolTime+ TimeUnit.SECONDS.toSeconds(5);
		endPooltime=initialPoolTime+ TimeUnit.SECONDS.toSeconds(5);
		ScoreInfo score = new ScoreInfo();
		if (map == null) {
			System.out.println("map is null at beginning of run");
		}
		try (ProgressBar pb = new ProgressBar("Progress:", 100, ProgressBarStyle.ASCII)) {
			long beginTime = events.peek().time;
			while (events.peek().time <= simulationEndTime) {
				if (tocheck.getClass() == ResourceEvent.class) {
				}
				Event toTrigger = events.poll();
				if(toTrigger.getClass()==ResourceEvent.class) resources++;
				pb.stepTo((long)(((float)(toTrigger.time - beginTime)) / (simulationEndTime - beginTime) * 100.0));
				if(toTrigger.getClass()==ResourceEvent.class && toTrigger.time>=initialPoolTime && toTrigger.time<endPooltime) {
					if(ResourceEvent.resList.isEmpty()) {
						continue;
					}
					createCostMatrix();
					AgentEvent.agentList.clear();
					ResourceEvent.resList.clear();
					initialPoolTime=initialPoolTime+ TimeUnit.SECONDS.toSeconds(5);
					endPooltime=initialPoolTime+ TimeUnit.SECONDS.toSeconds(5);
				}
				Event e = toTrigger.trigger();
				if (e != null) {
					events.add(e);
				}
			}
		} catch (Exception e) {
			System.out.println(resources);
			System.out.println(tocheck.id);
			System.out.println(tocheck.time);
			e.printStackTrace();
		}

		System.out.println("Simulation finished.");

		score.end();
	}

	/**
	 * This class is used to give a performance report and the score. It prints
	 * the total running time of the simulation, the used memory and the score.
	 * It uses Runtime which allows the application to interface with the
	 * environment in which the application is running.
	 */
	class ScoreInfo {

		Runtime runtime = Runtime.getRuntime();
		NumberFormat format = NumberFormat.getInstance();
		StringBuilder sb = new StringBuilder();

		long startTime;
		long allocatedMemory;

		/**
		 * Constructor for ScoreInfo class. Runs beginning, this method
		 * initializes all the necessary things.
		 */
		ScoreInfo() {
			startTime = System.nanoTime();
			// Suppress memory allocation information display
			// beginning();
		}

		/**
		 * Initializes and gets the max memory, allocated memory and free
		 * memory. All of these are added to the Performance Report which is
		 * saved in the StringBuilder. Furthermore also takes the time, such
		 * that later on we can compare to the time when the simulation is over.
		 * The allocated memory is also used to compare to the allocated memory
		 * by the end of the simulation.
		 */
		void beginning() {
			// Getting the memory used
			long maxMemory = runtime.maxMemory();
			allocatedMemory = runtime.totalMemory();
			long freeMemory = runtime.freeMemory();

			// probably unnecessary
			sb.append("Performance Report: " + "\n");
			sb.append("free memory: " + format.format(freeMemory / 1024) + "\n");
			sb.append("allocated memory: " + format.format(allocatedMemory / 1024)
					+ "\n");
			sb.append("max memory: " + format.format(maxMemory / 1024) + "\n");

			// still looking into this one "freeMemory + (maxMemory -
			// allocatedMemory)"
			sb.append("total free memory: "
					+ format.format(
					(freeMemory + (maxMemory - allocatedMemory)) / 1024)
					+ "\n");

			System.out.print(sb.toString());
		}

		/**
		 * Calculate the time the simulation took by taking the time right now
		 * and comparing to the time when the simulation started. Add the total
		 * time to the report and the score as well. Furthermore, calculate the
		 * allocated memory by the participant's implementation by comparing the
		 * previous allocated memory with the current allocated memory. Print
		 * the Performance Report.
		 */
		void end() {
			// Empty the string builder
			sb.setLength(0);

			long endTime = System.nanoTime();
			long totalTime = (endTime - startTime) / 1000000000;

			System.out.println("\nrunning time: " + totalTime);

			System.out.println("\n***Simulation environment***");
			System.out.println("JSON map file: " + mapJSONFile);
			System.out.println("Resource dataset file: " + resourceFile);
			System.out.println("Bounding polygon KML file: " + boundingPolygonKMLFile);
			System.out.println("Number of agents: " + totalAgents);
			System.out.println("Number of resources: " + totalResources);
			System.out.println("Resource Maximum Life Time: " + ResourceMaximumLifeTime + " seconds");
			System.out.println("Agent class: " + agentClass.getName());

			System.out.println("\n***Statistics***");

			if (totalResources != 0) {
				// Collect the "search" time for the agents that are empty at the end of the simulation.
				// These agents are in search status and therefore the amount of time they spend on
				// searching until the end of the simulation should be counted toward the total search time.
				long totalRemainTime = 0;
				for (AgentEvent ae: emptyAgents) {
					totalRemainTime += (simulationEndTime - ae.startSearchTime);
				}
				sb.append("Total simulation time: " + totalTime + " seconds \n");
				sb.append("average agent search time: " + Math.floorDiv(totalAgentSearchTime + totalRemainTime, (totalAssignments + emptyAgents.size())) + " seconds \n");
				sb.append("average resource wait time: " + Math.floorDiv(totalResourceWaitTime, totalResources) + " seconds \n");
				sb.append("resource expiration percentage: " + Math.floorDiv(expiredResources * 100, totalResources) + "%\n");
				sb.append("\n");
				System.out.println("Expired resources" + expiredResources);
//				sb.append("average agent cruise time: " + Math.floorDiv(totalAgentCruiseTime, totalAssignments) + " seconds \n");
//				sb.append("average agent approach time: " + Math.floorDiv(totalAgentApproachTime, totalAssignments) + " seconds \n");
//				sb.append("average resource trip time: " + Math.floorDiv(totalResourceTripTime, totalAssignments) + " seconds \n");
				sb.append("total number of assignments: " + totalAssignments + "\n");
				sb.append("total resouces: " + totalResources);
			} else {
				sb.append("No resources.\n");
			}

			System.out.print(sb.toString());
		}
	}

	/**
	 * Compares agent events
	 */
	class AgentEventComparator implements Comparator<AgentEvent> {

		/**
		 * Checks if two agentEvents are the same by checking their ids.
		 *
		 * @param a1 The first agent event
		 * @param a2 The second agent event
		 * @return returns 0 if the two agent events are the same, 1 if the id of
		 * the first agent event is bigger than the id of the second agent event,
		 * -1 otherwise
		 */
		public int compare(AgentEvent a1, AgentEvent a2) {
			if (a1.id == a2.id)
				return 0;
			else if (a1.id > a2.id)
				return 1;
			else
				return -1;
		}
	}

	/**
	 * Compares resource events
	 */
	class ResourceEventComparator implements Comparator<ResourceEvent> {
		/**
		 * Checks if two resourceEvents are the same by checking their ids.
		 *
		 * @param a1 The first resource event
		 * @param a2 The second resource event
		 * @return returns 0 if the two resource events are the same, 1 if the id of
		 * the resource event is bigger than the id of the second resource event,
		 * -1 otherwise
		 */
		public int compare(ResourceEvent a1, ResourceEvent a2) {
			if (a1.id == a2.id)
				return 0;
			else if (a1.id > a2.id)
				return 1;
			else
				return -1;
		}
	}

	/**
	 * Retrieves the total number of agents
	 *
	 * @return {@code totalAgents }
	 */
	public long totalAgents() {
		return totalAgents;
	}

	/**
	 * Retrieves the CityMap instance of this simulation
	 *
	 * @return {@code map }
	 */
	public CityMap getMap() {
		return map;
	}

	/**
	 * Sets the events of the simulation.
	 *
	 * @param events The PriorityQueue of events
	 */
	public void setEvents(PriorityQueue<Event> events) {
		this.events = events;
	}

	/**
	 * Retrieves the queue of events of the simulation.
	 *
	 * @return {@code events }
	 */
	public PriorityQueue<Event> getEvents() {
		return events;
	}

	/**
	 * Gets the empty agents in the simulation
	 *
	 * @return {@code emptyAgents }
	 */
	public TreeSet<AgentEvent> getEmptyAgents() {
		return emptyAgents;
	}

	/**
	 * Sets the empty agents in the simulation
	 *
	 * @param emptyAgents The TreeSet of agent events to set.
	 */
	public void setEmptyAgents(TreeSet<AgentEvent> emptyAgents) {
		this.emptyAgents = emptyAgents;
	}

	/**
	 * Make an agent copy of locationOnRoad so that an agent cannot modify the attributes of the road.
	 *
	 * @param locationOnRoad the location to make a copy for
	 * @return an agent copy of the location
	 */
	public LocationOnRoad agentCopy(LocationOnRoad locationOnRoad) {
		Intersection from = mapForAgents.intersections().get(locationOnRoad.road.from.id);
		Intersection to = mapForAgents.intersections().get(locationOnRoad.road.to.id);
		Road roadAgentCopy = from.roadsMapFrom.get(to);
		LocationOnRoad locationOnRoadAgentCopy = new LocationOnRoad(roadAgentCopy, locationOnRoad.travelTimeFromStartIntersection);
		return locationOnRoadAgentCopy;
	}
}
