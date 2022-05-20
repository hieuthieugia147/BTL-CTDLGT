
package multihop;

import multihop.request.RequestBase;

public class RTable {
	int id;
	String des;
	String route;
	int hop;
	double ratio;
	double timeCompute;
	double timeTrans;
	double resource;
	int npath;
	double cWL;
	RequestBase req;
	double timeSer;
	int sameDes;

	/**
	 * @param des      is node destination
	 * @param route    is next node
	 * @param hop      is number of hop from des
	 * @param resource is capacity of node
	 */
	public RTable(int id, String des, String route, int hop, double resource) {
		super();
		this.id = id;
		this.des = des;
		this.route = route;
		this.hop = hop;
		this.resource = resource;
		this.cWL = 0;
		this.sameDes = 1;
	}

	public RTable(int id, String des, String route, int hop, double resource, RequestBase req) {
		super();
		this.id = id;
		this.des = des;
		this.route = route;
		this.hop = hop;
		this.resource = resource;
		this.cWL = 0;
		this.req = req;
		this.sameDes = 1;
	}

	public int getsameDes() {
		return this.sameDes;
	}

	public void setsameDes(int sameDes) {
		this.sameDes = sameDes;
	}

	public double getTimeSer() {
		return timeSer;
	}

	public void setTimeSer(double timeSer) {
		this.timeSer = timeSer;
	}

	public RequestBase getReq() {
		return req;
	}

	public void setReq(RequestBase req) {
		this.req = req;
	}

	public double getcWL() {
		return cWL;
	}

	public void setcWL(double cWL) {
		this.cWL = cWL;
	}

	public int getNpath() {
		return npath;
	}

	public void setNpath(int npath) {
		this.npath = npath;
	}

	public int getId() {
		return id;
	}

	public void setId(int id) {
		this.id = id;
	}

	public String getDes() {
		return des;
	}

	public void setDes(String des) {
		this.des = des;
	}

	public String getRoute() {
		return route;
	}

	public void setRoute(String route) {
		this.route = route;
	}

	public int getHop() {
		return hop;
	}

	public void setHop(int hop) {
		this.hop = hop;
	}

	public double getRatio() {
		return ratio;
	}

	public void setRatio(double ratio) {
		this.ratio = ratio;
	}

	public double getTimeCompute() {
		return timeCompute;
	}

	public void setTimeCompute(double timeCompute) {
		this.timeCompute = timeCompute;
	}

	public double getTimeTrans() {
		return timeTrans;
	}

	public void setTimeTrans(double timeTrans) {
		this.timeTrans = timeTrans;
	}

	public double getResource() {
		return resource;
	}

	public void setResource(double resource) {
		this.resource = resource;
	}

	public String toString() {
		String rrtable = "id: " + id + " reqId:" + req.getId() + " src:" + req.getSrcNode().getName() + " reqTimeinit: "
				+ req.getTimeInit() + " des:" + des + " route:" + route + " sameDes:" + sameDes + " p(ratio):" + ratio
				+ " tComp:" + timeCompute + " tTrans:" + timeTrans + " tSer: " + (timeCompute + timeTrans)
				+ " hop:" + hop + " npath:" + npath
				+ " cWL:" + cWL
				+ " res:" + resource;
		return rrtable;
	}

}
