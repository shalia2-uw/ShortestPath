package roadgraph;

public class MapEdge {
	private MapNode start, end;
	private String streetName,roadType;
	private double distance;
	public MapEdge(MapNode start,MapNode end, String streetName, String roadType, double length){
		this.start = start;
		this.end = end;
		this.streetName = streetName;
		this.roadType = roadType;
		this.distance = length;
	}
	public MapNode getStart() {
		return start;
	}
	public MapNode getEnd() {
		return end;
	}
	public String getStreetName() {
		return streetName;
	}
	public double getDistance() {
		return distance;
	}
	public String getRoadType() {
		return roadType;
	}
	public MapNode getOtherNode(MapNode node){
		if(node.equals(start))
			return end;
		else if(node.equals(end))
			return start;
		else
			throw new IllegalArgumentException();
	}
}
