package roadgraph;

import java.util.HashSet;
import java.util.Set;

import geography.GeographicPoint;

@SuppressWarnings("rawtypes")
public class MapNode implements Comparable{
	private GeographicPoint location;
	private HashSet<MapEdge> edges;
	private double distance;
	private double currDist;
	
	public MapNode(GeographicPoint location){
		this.location = location;
		edges = new HashSet<>();
	}
	public double getDistance() {
		return distance;
	}
	public void setDistance(double distance) {
		this.distance = distance;
	}
	public double getCurrDist() {
		return currDist;
	}
	public void setCurrDist(double currDist) {
		this.currDist = currDist;
	}
	public GeographicPoint getLocation() {
		return location;
	}
	public void setLocation(GeographicPoint location) {
		this.location = location;
	}
	public HashSet<MapEdge> getEdges() {
		return edges;
	}
	public void addEdges(MapEdge edge) {
		edges.add(edge);
	}
	public Set<MapNode> getNeighbors(){
		Set<MapNode> neighbors = new HashSet<>();
		for(MapEdge e:edges){
			neighbors.add(e.getOtherNode(this));
		}
		return neighbors;
	}
	public double getDistanceToNode(MapNode node){
		for(MapEdge e:edges){
			if(e.getEnd().equals(node))
				return e.getDistance();
		}
		throw new IllegalArgumentException("There is no edge connecting to the provided node");
	}
	
	@Override
	public int compareTo(Object o) {
		// TODO Auto-generated method stub
		MapNode m = (MapNode)o;
		return ((Double)this.getDistance()).compareTo((Double)m.getDistance()); 
	}
}
