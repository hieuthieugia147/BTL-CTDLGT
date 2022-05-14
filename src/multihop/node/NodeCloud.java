package multihop.node;

import java.util.ArrayList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Vector;

import multihop.request.RequestCloud;

public class NodeCloud extends NodeBase {
	double CWL;
	double aWL;

	public NodeCloud(int id, String name, double lat, double lng, int range, double res) {
		super(id, name, lat, lng, range, res);
		// TODO Auto-generated constructor stub
	}

	Queue<RequestCloud> qReq = new PriorityQueue<RequestCloud>();
	List<RequestCloud> doneReq = new ArrayList<RequestCloud>();
	Queue<RequestCloud> qReqV = new PriorityQueue<RequestCloud>();

	public String toString() {

		return name + ": " + lat + " , " + lng;

	}

	public Queue<RequestCloud> getqReqV() {
		return qReqV;
	}

	public void setqReqV(Queue<RequestCloud> qReqV) {
		this.qReqV = qReqV;
	}

	public Queue<RequestCloud> getqReq() {
		return qReq;
	}

	public void setqReq(Queue<RequestCloud> qReq) {
		this.qReq = qReq;
	}

	public List<RequestCloud> getDoneReq() {
		return doneReq;
	}

	public void setDoneReq(List<RequestCloud> doneReq) {
		this.doneReq = doneReq;
	}

	public double getCWL() {
		return CWL;
	}

	public void setCWL(double cWL) {
		CWL = cWL;
	}

	public double getaWL() {
		return aWL;
	}

	public void setaWL(double aWL) {
		this.aWL = aWL;
	}
}
