package multihop.request;

import multihop.node.NodeCloud;
import multihop.node.NodeVehicle;

public class RequestCloud extends RequestVehicle {
    NodeCloud srcNodeCloud;
    int desCloud;
    double wlCloud;
    double timeRC;// t_trans from RSU to Cloud

    public RequestCloud(int id, double wL, NodeVehicle srcNode, double timeInit, boolean done, int des, String route,
            double start, double end) {
        super(id, wL, srcNode, timeInit, done, des, route, start, end);
    }

    public RequestCloud(RequestVehicle rv) {
        // dWL = rv.getMovedData();
        super(rv.getId(), rv.getMovedData(), rv.getSrcNode(), rv.getTimeInit(), rv.isDone(), rv.getDes(), rv.getRoute(),
                rv.getStart(), rv.getEnd());
        this.wlCloud = rv.getMovedData();
    }

    public String toString() {
        if (this.srcNodeCloud != null) {
            return "req" + this.id + " (p(ratio)=" + this.ratio + "\tsrcNodeCloud: " + this.srcNodeCloud.getName()
                    + "\tsrcNodeRSU: " + this.srcNode.getName() + ")";
        } else {
            return "req" + this.id + " (" + this.ratio + " " + this.srcNode.getName() + ")";
        }
    }

    public double getTimeRC() {
        return timeRC;
    }

    public void setTimeRC(double timeRC) {
        this.timeRC = timeRC;
    }

    public NodeCloud getSrcNodeCloud() {
        return srcNodeCloud;
    }

    public void setSrcNodeCloud(NodeCloud srcNodeCloud) {
        this.srcNodeCloud = srcNodeCloud;
    }

    public double getWlCloud() {
        return wlCloud;
    }

    public void setWlCloud(double wlCloud) {
        this.wlCloud = wlCloud;
    }

    @Override
    public int compareTo(RequestBase o) {
        if (this.start >= ((RequestCloud) o).start) {
            return 1;
        } else if (this.start < ((RequestCloud) o).start) {
            return -1;
        } else {
            return 0;
        }
    }
}
