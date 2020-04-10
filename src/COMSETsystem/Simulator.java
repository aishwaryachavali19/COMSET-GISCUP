package COMSETsystem;

import MapCreation.*;

import java.text.NumberFormat;
import java.util.*;
import java.util.logging.Level;
import java.util.logging.Logger;

//import com.sun.org.apache.xpath.internal.operations.Bool;
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

	// A class that extends BaseAgent and implements a search routing strategy
	protected final Class<? extends BaseAgent> agentClass;

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
		map.calcTravelDistances();
		// Make a map copy for agents to use so that an agent cannot modify the map used by
		// the simulator
		mapForAgents = map.makeCopy();

		MapWithData mapWD = new MapWithData(map, this.resourceFile, agentPlacementRandomSeed);

		// map match resources
		System.out.println("Loading and map-matching resources...");
		long latestResourceTime = mapWD.createMapWithData(this);

		// The simulation end time is the expiration time of the last resource.
		this.simulationEndTime = latestResourceTime;

		// Deploy agents at random locations of the map.
		System.out.println("Randomly placing " + this.totalAgents + " agents on the map...");
		agents = mapWD.placeAgentsRandomly(this);

		// Initialize the event queue.
		events = mapWD.getEvents();
	}

	/**
	 * My method to try and create preference lists of resources
	 */

	public void stableMarriage()
	{
		HashMap<Long, ArrayList<Long>> resPrefList= new HashMap<Long, ArrayList<Long>>();
		//HashMap<Long,HashMap<Long,Long> > resPrefList= new HashMap<Long, HashMap<Long,Long>>();

		HashMap<Long, Long>agentMatches=new HashMap<>();
		HashMap<Long, Long>resMatches=new HashMap<>();
		HashMap<Long,Integer>indexTrackForRes=new HashMap<>();

		//TODO Check if the arrival time is less than resource expiration and then add it to the preference list
		long min= Long.MAX_VALUE;
		for(Event event : events)
		{
			if(event.isResource())
			{
				resMatches.put(event.id,-1L);
				//waitingResources.;
				indexTrackForRes.put(event.id,0);
				LinkedHashMap<Long,Long> currentArrivalTime=new LinkedHashMap<>();
				for (AgentEvent agent : emptyAgents) {

					// Calculate the travel time from the agent's current location to resource.
					// Assumption: agent.time is the arrival time at the end intersection of agent.loc.road.
					// This assumption is true for empty agents. Notice that when agents are initially introduced
					// to the system, they are empty and agent.time is not necessarily the time to arrive at the end intersection.
					// However, all the agents are triggered once before the earliest resource (see MapWithData.createMapWithData).
					// When that happens, agent.time is updated to the end intersection arrival time.
					// Thus the assumption is still true.
					long travelTimeToEndIntersection = agent.time - ((ResourceEvent) event).time;
					long travelTimeFromStartIntersection = agent.loc.road.travelTime - travelTimeToEndIntersection;
					LocationOnRoad agentLocationOnRoad = new LocationOnRoad(agent.loc.road, travelTimeFromStartIntersection);
					long travelTime = map.travelTimeBetween(agentLocationOnRoad, ((ResourceEvent) event).pickupLoc);
					long arriveTime = travelTime +((ResourceEvent) event).time;
					if(arriveTime<((ResourceEvent) event).expirationTime)
					{
						currentArrivalTime.put(agent.id,arriveTime);
						//System.out.println("Added to customer pref list.");
					}


				}
				PriorityQueue<Map.Entry<Long, Long>> pq = new PriorityQueue<>((a,b) -> a.getValue()>b.getValue()?1:-1);
				pq.addAll(currentArrivalTime.entrySet());
				//currentArrivalTime.clear();
				ArrayList<Long> aList=new ArrayList<Long>();
				for(int i=0;i<pq.size();i++)
				{
					if(pq.isEmpty())
						break;
					Map.Entry<Long,Long> e=pq.poll();
					//currentArrivalTime.put(e.getKey(),e.getValue());
					aList.add(e.getKey());
				}
				//resPrefList.put(event.id,currentArrivalTime);
				resPrefList.put(event.id,aList);

			}

		}

		//Calculate the preference list for agents

		//HashMap<Long,HashMap<Long,Double> > agentPrefList= new HashMap<Long, HashMap<Long,Double>>();
		HashMap<Long, ArrayList<Long>> agentPrefList= new HashMap<Long, ArrayList<Long>>();
		for(AgentEvent agent: emptyAgents)
		{
			agentMatches.put(agent.id,-1L);
			HashMap<Long,Double> currentBenefit=new HashMap<>();
			for(Event event : events)
			{
				if(event.isResource())
				{

					long travelTimeToEndIntersection = agent.time - ((ResourceEvent) event).time;
					long travelTimeFromStartIntersection = agent.loc.road.travelTime - travelTimeToEndIntersection;
					LocationOnRoad agentLocationOnRoad = new LocationOnRoad(agent.loc.road, travelTimeFromStartIntersection);
					long travelTime = map.travelTimeBetween(agentLocationOnRoad, ((ResourceEvent) event).pickupLoc);
					long arriveTime = travelTime +((ResourceEvent) event).time;

					long distance=map.travelDistanceBetween(agentLocationOnRoad,((ResourceEvent) event).pickupLoc);
					long tripDistance=map.travelDistanceBetween(((ResourceEvent) event).pickupLoc,((ResourceEvent) event).dropoffLoc);
					double benefit= (double)tripDistance/(tripDistance + distance);
					if( arriveTime<((ResourceEvent) event).expirationTime)
					{
						currentBenefit.put(event.id,benefit);
						//System.out.println("Added to resource pref list.");
					}

				}
			}
			PriorityQueue<Map.Entry<Long, Double>> pq2 = new PriorityQueue<>((a,b) -> a.getValue()>b.getValue()?-1:1);
			pq2.addAll(currentBenefit.entrySet());
			//currentBenefit.clear();
			ArrayList<Long> aList2=new ArrayList<Long>();
			for(int i=0;i<pq2.size();i++)
			{
				if(pq2.isEmpty())
					break;
				Map.Entry<Long,Double> e=pq2.poll();
				//currentBenefit.put(e.getKey(),e.getValue());
				aList2.add(e.getKey());
				//System.out.print(" "+"("+e.getKey()+","+e.getValue()+")");
			}
			//agentPrefList.put(agent.id,currentBenefit);
			agentPrefList.put(agent.id,aList2);

		}
		/*
		//To print agent preference list along with the benefit
		System.out.println("Agent pref list:");
		for(HashMap<Long,Double> innerList : agentPrefList.values()) {

			for(Long number : innerList.keySet()) {
				System.out.print(number+":" +innerList.get(number)+",");
			}
			System.out.println();
		}
		*/
		//Agent preference list with only resource IDs
		System.out.println("Agent pref list:");

		/*for(Long key: agentPrefList.keySet()){
			//ArrayList<Long> innerList = agentPrefList.get(key);
			System.out.println("Keys:"+key);
		}*/
		//agent IDs for pref list
		int j=1;
		ArrayList<Long> agentKeys = new ArrayList<>();
		for(Long key: agentPrefList.keySet()) {
			agentKeys.add(key);
		}
		//Printing
		for(ArrayList<Long> innerList : agentPrefList.values()) {
				if(!innerList.isEmpty()) {
					//System.out.println("inside !innerList.isEmpty()"+"  agentKeys.size():"+agentKeys.size() +"agentPreflist.size():"+agentPrefList.size() );
					if(j<agentKeys.size()) {
						//System.out.println("inside !j<agentKeys.size()");
						System.out.println("Agent:" + agentKeys.get(j++));
					}
					for (Long number : innerList) {
						System.out.print(number + ",");
					}

					System.out.println();
				}
		}
		/*
		//To print resource preference list along with waiting time. Need to change code a little to get the required o/p
		System.out.println("\n\nResource pref list:");
		for(HashMap<Long,Long> innerList : resPrefList.values()) {

			for(Long number : innerList.keySet()) {
				System.out.print(number+":" +innerList.get(number)+",");
			}
			System.out.println();
		}
		*/

		//Resource preference list with only agent IDs
		System.out.println("\n\nResource pref list:");
		for(ArrayList<Long> innerList : resPrefList.values()) {

			for(Long number : innerList) {
				System.out.print(number+",");
			}
			System.out.println();
		}

		//Stable Matching algorithm


		int freeRes=resMatches.size();
		while(freeRes >0)
		{
			//System.out.println("freeRes:"+freeRes);

			for(Long resId: resMatches.keySet()) //For all the resources
			{
				int maxSize=resPrefList.get(resId).size();
				if((resMatches.get(resId)==-1L) && (indexTrackForRes.get(resId)!=maxSize)) //The resource is free and has some agent left to propose
				{
					ArrayList<Long>PrefList=resPrefList.get(resId);
					int i=indexTrackForRes.get(resId);
					while(i<PrefList.size())
					{
						Long agentId=PrefList.get(i);
						if(agentMatches.get(agentId)==-1L) //The agent is free. Then match the resource with that agent.
						{
							resMatches.put(resId,agentId);

							agentMatches.put(agentId,resId);
							indexTrackForRes.put(resId,(i+1));
							freeRes--;
							break;

						}
						else							//Agent is already matched to another resource. Check if the current res is preferred
						{
							ArrayList<Long>agentPref=agentPrefList.get(agentId);
							Long currentMatchedRes=agentMatches.get(agentId);
							int indexCurrentMatch=agentPref.indexOf(currentMatchedRes);
							int index=agentPref.indexOf(resId);
							if((index<indexCurrentMatch && index!=-1) || (indexCurrentMatch==-1 && index!=indexCurrentMatch))
							{
								resMatches.put(resId,agentId);
								resMatches.put(currentMatchedRes,-1L); //Mark the previous resource as free
								//waitingResources.add();
								indexTrackForRes.put(resId,i+1);
								agentMatches.put(agentId,resId);
								break;
							}
						}
						i++;
						if(i==maxSize)   //If all the agents on the preference list are exhausted
						{
							indexTrackForRes.put(resId,maxSize);
							freeRes--;
						}

					}
				}

			}
		}
		System.out.println("The matching is as follows for resources:");
		int numberOfResMatched=0;
		for(Long r_id : resMatches.keySet() ) {

			if(resMatches.get(r_id)!=-1L)
			{
				numberOfResMatched++;
			}
			System.out.print(r_id+","+resMatches.get(r_id));

			System.out.println();
		}
		System.out.println("The number of resources matched:"+numberOfResMatched+" percentage:"+(numberOfResMatched/resPrefList.size()));

		System.out.println("The matching is as follows for agents:");
		for(Long r_id : agentMatches.keySet() ) {


			System.out.print(r_id+","+agentMatches.get(r_id));

			System.out.println();
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
		System.out.println("Running the simulation...");

		ScoreInfo score = new ScoreInfo();
		if (map == null) {
			System.out.println("map is null at beginning of run");
		}
		try (ProgressBar pb = new ProgressBar("Progress:", 100, ProgressBarStyle.ASCII)) {
			long beginTime = events.peek().time;
			while (events.peek().time <= simulationEndTime) {
				Event toTrigger = events.poll();
				pb.stepTo((long)(((float)(toTrigger.time - beginTime)) / (simulationEndTime - beginTime) * 100.0));
				Event e = toTrigger.trigger();
				if (e != null) {
					events.add(e);
				}
			}
		} catch (Exception e) {
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

				sb.append("average agent search time: " + Math.floorDiv(totalAgentSearchTime + totalRemainTime, (totalAssignments + emptyAgents.size())) + " seconds \n");
				sb.append("average resource wait time: " + Math.floorDiv(totalResourceWaitTime, totalResources) + " seconds \n");
				sb.append("resource expiration percentage: " + Math.floorDiv(expiredResources * 100, totalResources) + "%\n");
				sb.append("\n");
				sb.append("average agent cruise time: " + Math.floorDiv(totalAgentCruiseTime, totalAssignments) + " seconds \n");
				sb.append("average agent approach time: " + Math.floorDiv(totalAgentApproachTime, totalAssignments) + " seconds \n");
				sb.append("average resource trip time: " + Math.floorDiv(totalResourceTripTime, totalAssignments) + " seconds \n");
				sb.append("total number of assignments: " + totalAssignments + "\n");
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