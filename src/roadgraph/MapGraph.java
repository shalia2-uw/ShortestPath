/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;
import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 3
	private HashMap<GeographicPoint, MapNode> vertices;
	private HashSet<MapEdge> edges;
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 3
		vertices = new HashMap<>();
		edges = new HashSet<>();
	}
		/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 3
		return vertices.values().size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 3
		return new HashSet<>(vertices.keySet());
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 3
		return edges.size();
	}
	public void addVertex(double lat, double lon){
		GeographicPoint loc = new GeographicPoint(lat, lon);
		this.addVertex(loc);
	}
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method in WEEK 3
		MapNode node = vertices.get(location);
		if(node== null){
			node = new MapNode(location);
			vertices.put(location, node);
			return true;
		}else{
			System.out.println("Node already exists");
			return false;
		}
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {
		//TODO: Implement this method in WEEK 3
		MapNode start = vertices.get(from);
		MapNode end = vertices.get(to);
		
		if(start.equals(null)||end.equals(null)||roadName.equals(null)||roadType.equals(null)||length<=0)
			throw new IllegalArgumentException();
		else if(!(vertices.containsKey(from))||!(vertices.containsKey(to)))
			throw new IllegalArgumentException();
		
		MapEdge me = new MapEdge(start, end, roadName, roadType, length);
		edges.add(me);
		start.addEdges(me);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		MapNode startNode = vertices.get(start);
		MapNode endNode = vertices.get(goal);
		Queue<MapNode> toExplore = new LinkedList<>();
		HashSet<MapNode> visited = new HashSet<>();
		HashMap<MapNode, MapNode> parent = new HashMap<>();
		toExplore.add(startNode);
		visited.add(startNode);
		MapNode curr=null;
		while(!(toExplore.isEmpty())){
			curr = toExplore.remove();
			// Hook for visualization.  See writeup.
			nodeSearched.accept(curr.getLocation());
			if(curr.equals(endNode))
				break;
			for(MapNode neighbor:getNeighbors(curr)){
				if(!visited.contains(neighbor)){
					visited.add(neighbor);
					parent.put(neighbor, curr);
					toExplore.add(neighbor);
				}
			}
		}
		return getPath(parent, startNode, endNode, curr.equals(endNode));
	}
	
	private Set<MapNode> getNeighbors(MapNode node) {
		return node.getNeighbors();
	}
	
	private List<GeographicPoint> getPath(HashMap<MapNode, MapNode> parent,MapNode start, MapNode goal, boolean foundPath){
		if(!foundPath){
			System.out.println("No path found from "+start.getLocation()+" to "+goal.getLocation());
			return new ArrayList<>();
		}
		LinkedList<GeographicPoint> path = new LinkedList<>();
		MapNode curr = goal;
		while(!curr.equals(start)){
			path.addFirst(curr.getLocation());
			curr = parent.get(curr);
		}
		path.addFirst(start.getLocation());
		return path;
	}
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4
		if(start==null||goal==null)
			throw new NullPointerException();
		MapNode startNode = vertices.get(start);
		MapNode endNode = vertices.get(goal);
		if(startNode==null||endNode==null)
			return null;
		
		PriorityQueue<MapNode> queue = new PriorityQueue<>();
		HashSet<MapNode> visited = new HashSet<>();
		HashMap<MapNode, MapNode> parent = new HashMap<>();
		double dist = 9999999999d;
		for(MapNode node:vertices.values()){
			node.setDistance(dist);
		}
		startNode.setDistance(0d);
		queue.add(startNode);
		MapNode curr=null;
		while(!queue.isEmpty()){
			curr=queue.remove();
//			System.out.println("DIJKSTRA visiting[NODE at location ("+curr.getLocation()+") intersects streets:");
//			System.out.println(hashToString(curr));
			// Hook for visualization.  See writeup.
			nodeSearched.accept(curr.getLocation());
			if(!visited.contains(curr)){
				visited.add(curr);
				if(curr.equals(endNode)){
					System.out.println("Dijkstra's:"+visited.size());
					return getPath(parent, startNode, endNode, curr.equals(endNode));
				}
				Set<MapNode> neighbors = getNeighbors(curr);
				for(MapNode neighbor:neighbors){
					if(!visited.contains(neighbor)){
						double neighborDist = curr.getDistance()+curr.getDistanceToNode(neighbor);
						double estimated = neighborDist + getDistanceToGoal(neighbor, endNode);
						if(neighborDist<neighbor.getDistance()){
							neighbor.setCurrDist(neighborDist);
							neighbor.setDistance(estimated);
							parent.put(neighbor, curr);
							queue.add(neighbor);
						}
					}
				}
			}
		}
		System.out.println("No path found from "+startNode.getLocation()+ "to "+endNode.getLocation());
		return null;
	}
	public double getDistanceToGoal(MapNode node, MapNode goal) {
		return Math.sqrt(Math.pow(goal.getLocation().x - node.getLocation().x, 2) + Math.pow(goal.getLocation().y - node.getLocation().y, 2));
	}
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4
		if(start==null||goal==null)
			throw new NullPointerException();
		MapNode startNode = vertices.get(start);
		MapNode endNode = vertices.get(goal);
		if(startNode==null||endNode==null)
			return null;
		
		PriorityQueue<MapNode> queue = new PriorityQueue<>();
		HashSet<MapNode> visited = new HashSet<>();
		HashMap<MapNode, MapNode> parent = new HashMap<>();
		double dist = 9999999999d;
		
		for(MapNode node:vertices.values()){
			node.setDistance(dist);
			node.setCurrDist(dist);
		}
		
		startNode.setDistance(0d);
		startNode.setCurrDist(0d);
		
		queue.add(startNode);
		MapNode curr = null;
		
		while(!queue.isEmpty()){
			curr = queue.remove();
//			System.out.println("A* visiting[NODE at location ("+curr.getLocation()+") intersects streets:");
//			System.out.println(hashToString(curr));
			// Hook for visualization.  See writeup.
			nodeSearched.accept(curr.getLocation());
			if(!visited.contains(curr)){
				visited.add(curr);
				
				if(curr.equals(endNode)){
					System.out.println("A*'s:"+visited.size());
					return getPath(parent, startNode, endNode, curr.equals(endNode));
				}
				
				Set<MapNode> neighbors = getNeighbors(curr);
				for(MapNode neighbor: neighbors){
					if(!visited.contains(neighbor)){
					
						double distanceToNeigbor = curr.getDistance()+ curr.getDistanceToNode(neighbor);
						if(distanceToNeigbor<neighbor.getDistance()){
							neighbor.setDistance(distanceToNeigbor);
							parent.put(neighbor, curr);
							queue.add(neighbor);
						}
					}
				}
			}
			
		}
		System.out.println("No path found from "+startNode.getLocation()+" to "+endNode.getLocation());
		return null;
	}
	
	public List<String> hashToString(MapNode curr){
		List<String> tmp = new ArrayList<>();
		for(MapEdge e: edges){
			if(e.getEnd().equals(curr)||e.getStart().equals(curr)){
				tmp.add(e.getStreetName());
			}
		}
		return tmp;
	}
	
	
	public static void main(String[] args)
	{
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
			
			GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
			GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
			
			System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
			List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
			List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
			
			MapGraph testMap = new MapGraph();
			GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
			
			// A very simple test using real data
			testStart = new GeographicPoint(32.869423, -117.220917);
			testEnd = new GeographicPoint(32.869255, -117.216927);
			System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
			testroute = testMap.dijkstra(testStart,testEnd);
			testroute2 = testMap.aStarSearch(testStart,testEnd);
			
			
			// A slightly more complex test using real data
			testStart = new GeographicPoint(32.8674388, -117.2190213);
			testEnd = new GeographicPoint(32.8697828, -117.2244506);
			System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
			testroute = testMap.dijkstra(testStart,testEnd);
			testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		/*MapGraph1 theMap = new MapGraph1();
		System.out.print("DONE. \nLoading the map...");
//		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);

		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);*/
	}
	
}
