package multihop.node;

import java.util.ArrayList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Vector;

import multihop.request.RequestVehicle;

public class NodeVehicle extends NodeBase {

	public NodeVehicle(int id, String name, double lat, double lng, int range, double res) {
		super(id, name, lat, lng, range, res);
		// TODO Auto-generated constructor stub
	}

	double[] x;
	double[] y;
	double[] velo;
	double[] phi;
	double[] sign;

	double cWL = 0; // current workload
	double aWL = 0; // assigned workload

	Vector<Vector<NodeVehicle>> nodeNeighbor = new Vector<Vector<NodeVehicle>>();

	Queue<RequestVehicle> qReq = new PriorityQueue<RequestVehicle>();

	List<RequestVehicle> doneReq = new ArrayList<RequestVehicle>();

	public Boolean checkLK(NodeVehicle a, int i) {
		double ax = a.getX()[i];
		double ay = a.getY()[i];
		double k = (this.x[i] - ax) * (this.x[i] - ax) + (this.y[i] - ay) * (this.y[i] - ay);
		if (k <= RANGE) {
			return true;
		}
		return false;
	}

	public String toString() {
		return name + ": x= " + lat + " , y = " + lng;
	}

	public Queue<RequestVehicle> getqReq() {
		return qReq;
	}

	public void setqReq(Queue<RequestVehicle> qReq) {
		this.qReq = qReq;
	}

	public List<RequestVehicle> getDoneReq() {
		return doneReq;
	}

	public void setDoneReq(List<RequestVehicle> doneReq) {
		this.doneReq = doneReq;
	}

	public Vector<Vector<NodeVehicle>> getNodeNeighbor() {
		return nodeNeighbor;
	}

	public void setNodeNeighbor(Vector<Vector<NodeVehicle>> nodeNeighbor) {
		this.nodeNeighbor = nodeNeighbor;
	}

	public Boolean checkLK(NodeVehicle a) {
		return null;
	}

	public double[] getX() {
		return x;
	}

	public void setX(double[] x) {
		this.x = x;
	}

	public double[] getY() {
		return y;
	}

	public void setY(double[] y) {
		this.y = y;
	}

	public double[] getVelo() {
		return velo;
	}

	public void setVelo(double[] velo) {
		this.velo = velo;
	}

	public double[] getPhi() {
		return phi;
	}

	public void setPhi(double[] phi) {
		this.phi = phi;
	}

	public double[] getSign() {
		return sign;
	}

	public void setSign(double[] sign) {
		this.sign = sign;
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
