package multihop.util;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import PSOSim.PSOSwarm;
import PSOSim.PSOVector;
import multihop.Constants;
import multihop.RTable;

public class AlgUtils {

	public static HashMap<Integer, Double> getPSO(List<RTable> rtable, HashMap<Integer, List<RTable>> mapRTable,
			int testCase, double ts) {
		HashMap<Integer, Double> result = new HashMap<Integer, Double>();
		int num = rtable.size();
		int particles = Constants.particles;
		int epchos = Constants.epchos;
		int dim = num; // number of nodes/dimension

		PSOSwarm swarm = new PSOSwarm(particles, epchos, dim, rtable, mapRTable, testCase);

		System.out.println("Running PSO in ts = " + ts);
		Map<Integer, Double> ratio = swarm.run("service-id-string");

		result = (HashMap<Integer, Double>) ratio;

		return result;
	}

	public AlgUtils() {
		// TODO Auto-generated constructor stub
	}

}
