package multihop.node;

public class NodeCloud extends NodeBase {
	int id;
	String name;

	double lat;
	double lng;

	double res;
	double aWL;
	double cWL;

	public NodeCloud(int id, String name, double lat, double lng, int range, double res) {
		super(id, name, lat, lng, range, res);
		// TODO Auto-generated constructor stub
	}

	public int getId() {
		return id;
	}

	public void setId(int id) {
		this.id = id;
	}

	public String getName() {
		return name;
	}

	public void setName(String name) {
		this.name = name;
	}

	public void setLat(double lat) {
		this.lat = lat;
	}

	public double getLng() {
		return lng;
	}

	public double getRes() {
		return res;
	}

	public void setRes(double res) {
		this.res = res;
	}

	public double getaWL() {
		return aWL;
	}

	public void setaWL(double aWL) {
		this.aWL = aWL;
	}

	public double getcWL() {
		return cWL;
	}

	public void setcWL(double cWL) {
		this.cWL = cWL;
	}

}
