package PSOSim;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Set;

import multihop.Constants;
import multihop.LogPSO;
import multihop.RTable;

class PSOFunction {

	private static final double INFINITY = Double.POSITIVE_INFINITY;
	private int nodes;
	private int testCase;

	public PSOFunction(int nodes, int testCase) {
		this.nodes = nodes;
		this.testCase = testCase;

	}

	/**
	 * @return PSOVector as value: vector {time(p)}
	 */

	public PSOVector multiFunction(PSOParticle p, List<RTable> rtable,
			HashMap<Integer, List<RTable>> mapRTable) {

		/**
		 * @param workLoad        number of images from 1 service
		 * @param p               percent workload
		 * @param Theta           share service or not ( 0 or 1)
		 * @param currentWorkload current workload at that node
		 * @return total time
		 */

		int workLoad;
		String pid = p.getName();
		// PSOVector personalBest = p.getBestPosition();
		PSOVector time = new PSOVector(p.getPosition().getDim()); // create a new psoVector same value

		PSOVector ratio = p.getPosition().getVectorRatio(); // ratio, get p of PSO vector

		// calcTimeSerMulti(rtable, position, worker, workLoad);
		// TODO position is time_server, rtable is list of node including virtual node
		// must be assigned
		// 0 Man Man 0
		// 1 W1 Man 1
		// 2 W3 W1 2
		// 3 W5 W1 2
		// 4 W2 Man 1
		// 5 W4 W2 2
		// 6 W5 W2 2

		// System.out.println("particle: " + p.getPosition().toStringOutput());
		double CWL = 0;

		if (testCase == 2) {
			// calc t_compute (none- t_wait)
			int jj = 0;
			for (RTable r : rtable) {
				workLoad = (int) r.getReq().getWL();
				double t_compute = 0;
				double subWL = ratio.getById(jj) * workLoad; // new WL
				double totalWL = subWL + r.getcWL(); // adding cWL
				t_compute = totalWL / r.getResource();
				r.setTimeCompute(t_compute);

				jj++;
			}
		} else {
			/**
			 * calc t_compute (include t_wait) output: TimeCompute, CWL
			 */
			List<String> check = new ArrayList<String>();
			int j = 0;
			for (RTable r : rtable) {
				workLoad = (int) r.getReq().getWL();
				double t_compute = 0;
				double subWL = ratio.getById(j) * workLoad; // new WL
				double totalWL = subWL + r.getcWL(); // adding cWL
				t_compute = totalWL / r.getResource();

				if (!check.contains(r.getDes())) {

					List<Integer> r2Id = new ArrayList<Integer>();
					int j2 = 0;
					for (RTable r2 : rtable) {
						if ((r2.getDes().equals(r.getDes()))
								&& (r2.getId() != r.getId() || (r2.getReq().getId() != r.getReq().getId()))) {
							r2Id.add(j2);
							t_compute += ratio.getById(j2) * r2.getReq().getWL() / r2.getResource();

						}
						j2++;
					}

					for (Integer i : r2Id) {
						rtable.get(i).setTimeCompute(t_compute);
					}
					r.setTimeCompute(t_compute);
				}
				check.add(r.getDes());
				j++;
			}
		}

		// Adding tran
		int j2 = 0;
		double TMAX = 0;

		for (RTable r : rtable) {
			double t_trans;

			t_trans = (ratio.getById(j2) * r.getReq().getWL() / Constants.BW) * r.getHop();
			if (r.getId() == 0)
				t_trans = 0;

			r.setTimeTrans(t_trans);
			// position.setById(j, r.getTimeTrans() + r.getTimeCompute());

			TMAX = r.getReq().getWL() / Constants.RES[Constants.TYPE.VEHICLE.ordinal()];

			time.setById(j2, (r.getTimeCompute() + r.getTimeTrans()) / TMAX);

			j2++;
		}

		// System.out.println(pid+ " Time calc in func: \n" + time.toStringOutput());

		int j3 = 0;
		for (RTable r : rtable) {
			TMAX = r.getReq().getWL() / Constants.RES[Constants.TYPE.VEHICLE.ordinal()];
			double check1 = (r.getTimeCompute() + r.getTimeTrans()) / TMAX - time.getById(j3);
			// System.out.println("rtable: " + j3 + " " + r.getTimeCompute());
			if (check1 != 0) {
				// System.out.println("Fail: " + j3 + " " + r.getDes() + " " + check1);
				// System.out.println(worker.toStringOutput());

			}

			j3++;
		}
		// System.out.println(worker.toStringOutput());

		int totalworkLoad = 0;
		// for (Integer id : mapRTable.keySet()) {
		// totalworkLoad += mapRTable.get(id).get(0).getReq().getWL();
		// }

		// System.out.println("BEFORE CONT\n" + position.toStringOutput());
		List<String> f2 = new ArrayList<String>();
		f2.add(constraintF2(time, pid, ratio));

		if ((testCase == 6) || (testCase == 7)) {
			// constraintF5(totalworkLoad, time, pid, rtable, ratio, mapRTable, testCase);
		}

		// update 3/9 for new constraint
		if ((testCase == 1) || (testCase == 2)) {
			constraintF5(totalworkLoad, time, pid, rtable, ratio, mapRTable, testCase, p);
			constraintF6(totalworkLoad, time, CWL, rtable, ratio, mapRTable, testCase);
		}

		// double TMAX= workLoad/Constants.BW+workLoad/Constants.RES;
		// double TMAX= workLoad/Constants.RES;

		// double A=Constants.A;
		// position.div(TMAX/A);

		// constraintF3(totalworkLoad / check.size(), currentWorkload, position, CWL,
		// mapRTable, worker, rtable);
		// constraintF4(totalworkLoad, position, CWL, rtable, worker, mapRTable);

		//
		// FileWriter myWriterPSO;
		// try {
		// myWriterPSO = new FileWriter("topoPSO_log");
		// myWriterPSO.append(position.toStringOutput());
		// myWriterPSO.close();
		// } catch (IOException e) {
		// // TODO Auto-generated catch block
		// e.printStackTrace();
		// }

		// System.out.println("RATIO: " + worker.toStringOutput());
		// System.out.println("POS: " + position.toStringOutput());

		// System.out.println(pid+ " Time in func: \n" + time.toStringOutput());
		return time;

	}

	static String constraintF2(PSOVector time, String pid, PSOVector ratio) {
		boolean c = false;
		for (int i = 0; i < time.getVectorCoordinate().length; i++) {
			if (ratio.getById(i) < 0) {
				// System.out.println("F2 at " + pid + " " + ratio.toStringOutput() + "\n" +
				// ratio.getById(i));
				time.setById(0, Constants.MAXDOUBLE);
				return pid;
			}
		}
		return null;
	}

	static PSOVector constraintF5(int workLoad, PSOVector time, String pid, List<RTable> rtable, PSOVector ratio,
			HashMap<Integer, List<RTable>> mapRTable, int testCase, PSOParticle p) {
		int j = 0;
		double addPen = 0;
		double check = 9999;
		for (RTable r : rtable) {
			double pai = ratio.getById(j);
			double lambWL = r.getResource() / r.getReq().getWL();
			// double t = worker.getById(j)*r.getReq().getWL()/r.getResource(); // time
			// process
			// double t = (worker.getById(j)* r.getReq().getWL() + r.getcWL()) /
			// r.getResource(); // time process include
			// // cWL
			// int j2 = 0;
			// for (RTable r2 : rtable) {
			// if ((r2.getDes().equals(r.getDes()))
			// && (r2.getId() != r.getId() || (r2.getReq().getId() != r.getReq().getId())))
			// {
			// t += worker.getById(j2)*r2.getReq().getWL()/r2.getResource(); // sum up
			// time_process in node: r.getDes()
			// }
			// j2++;
			// }
			// double A= Constants.A;
			// if(testCase==7) {
			// A=0.1;
			// }
			double pen = pai - lambWL;

			if (!r.getDes().equals(r.getReq().getSrcNode().getName())) { // khong xet req node trong dieu kien nay
				// pen = pai - lambWL;
				if (pen > 0) {
					pen = INFINITY;
					time.setById(j, time.getById(j) + pen);
					check = j;
					// System.out.println("pa->i: " + pai + " " + r.getReq().getSrcNode().getName()
					// + "->" + r.getDes());
					// addPen += pen;
					// System.out.println("F3 "+ pid + " :" + ratio.toStringOutput() + "\n" +
					// r.getReq().getId() + " " + j);
					// System.out.println(p.getBestPosition().toStringOutput());
					// System.out.println("j pa->i: " + j + " p" + r.getReq().getSrcNode().getName()
					// + "->" + r.getDes() + " :" + pai);
					// System.out.println(r.toString());

				}
			}
			// addPen += pen;
			j++;

		}

		if (check != 9999) {
			LogPSO log = LogPSO.getInstance();
			// log.log("\tF3." + j + "\t");
			// log.log("\n" +worker.toStringOutput() + "\n");
		}

		for (int i = 0; i < time.getDim(); i++) {
			// postion.setById(i, postion.getById(i)+ addPen);
		}

		// for (RTable r : rtable) {
		// double t = worker.getById(j)*r.getReq().getWL()/r.getResource(); // time
		// process
		// //double t =
		// (worker.getById(j)+r.getcWL())*r.getReq().getWL()/r.getResource(); // time
		// process include cWL
		// int j2 = 0;
		// for (RTable r2 : rtable) {
		// if ((r2.getDes().equals(r.getDes()))
		// && (r2.getId() != r.getId() || (r2.getReq().getId() != r.getReq().getId())))
		// {
		// t += worker.getById(j2)*r2.getReq().getWL()/r2.getResource(); // sum up
		// time_process in node: r.getDes()
		// }
		// j2++;
		// }
		// double A= Constants.A;
		// if(testCase==7) {
		// A=0.1;
		// }

		// double pen = t/(Constants.TS*(1+A));
		// if (pen > 1) {
		// postion.setById(j, postion.getById(j) + pen);
		// }

		// //sump.put(nodeID, p);
		// j++;
		// }
		return time;
	}

	static PSOVector constraintF6(int workLoad, PSOVector time, double cWL, List<RTable> rtable, PSOVector ratio,
			HashMap<Integer, List<RTable>> mapRTable, int testCase) {
		int j = 0;
		double addPen = 0;
		double check = 9999;
		List<Integer> checked = new ArrayList<Integer>();
		for (RTable r : rtable) {

			double t_ser = 0;
			int reqID = r.getReq().getId();
			if (!checked.contains(reqID)) {
				double t_ser_r = 0;
				double WLlamb = r.getReq().getWL() / r.getResource();
				if (!r.getDes().equals(r.getReq().getSrcNode().getName())) {
					t_ser_r = time.getById(j);
				}
				int j2 = 0;
				for (RTable r2 : rtable) {
					double t_ser_r2 = time.getById(j2);
					if ((r2.getReq().getId() == reqID) && (r2.getId() != r.getId())) {
						t_ser = (t_ser_r > t_ser_r2) ? t_ser_r : t_ser_r2;
					}
					j2++;
				}

				double pen = t_ser - WLlamb;

				if (pen > 0) {
					pen = INFINITY;
					time.setById(j, t_ser + pen);
					// addPen += pen;
					check = j;
				}
			}
			checked.add(reqID);
			j++;

		}

		if (check != 9999) {
			LogPSO log = LogPSO.getInstance();
			// System.out.println("F4: " + ratio.toStringOutput()) ;
			// log.log("\tF4." + j + "\t");
		}

		for (int i = 0; i < time.getDim(); i++) {
			// postion.setById(i, postion.getById(i)+ addPen);
		}

		// for (RTable r : rtable) {
		// double t = worker.getById(j)*r.getReq().getWL()/r.getResource(); // time
		// process
		// //double t =
		// (worker.getById(j)+r.getcWL())*r.getReq().getWL()/r.getResource(); // time
		// process include cWL
		// int j2 = 0;
		// for (RTable r2 : rtable) {
		// if ((r2.getDes().equals(r.getDes()))
		// && (r2.getId() != r.getId() || (r2.getReq().getId() != r.getReq().getId())))
		// {
		// t += worker.getById(j2)*r2.getReq().getWL()/r2.getResource(); // sum up
		// time_process in node: r.getDes()
		// }
		// j2++;
		// }
		// double A= Constants.A;
		// if(testCase==7) {
		// A=0.1;
		// }

		// double pen = t/(Constants.TS*(1+A));
		// if (pen > 1) {
		// postion.setById(j, postion.getById(j) + pen);
		// }

		// //sump.put(nodeID, p);
		// j++;
		// }
		return time;
	}

	/**
	 * Sum pi = 1
	 * 
	 * @param p         particle
	 * @param mapRTable
	 * @return true if not satisfy
	 */
	static boolean constraintF1(PSOParticle p, HashMap<Integer, List<RTable>> mapRTable) {
		// System.out.println(p.getPosition().toStringOutput());
		LogPSO log = LogPSO.getInstance();
		int j = 0;
		Set<Integer> keySet = mapRTable.keySet();
		List<Integer> sortedList = new ArrayList<>(keySet);
		Collections.sort(sortedList);

		for (Integer id : sortedList) { // req 0, 1
			double checkSum = 0;

			List<RTable> rTable = mapRTable.get(id);

			for (int i = 0; i < rTable.size(); i++) {
				// System.out.println(p.getPosition().getById(j));
				checkSum += p.getPosition().getById(j);
				j++;
			}
			int check = (int) (checkSum * (10000)) - 10000;
			// System.out.println("checkSum = " + checkSum + " " + check);

			if (Math.abs(check) > 1) {
				// System.out.println("F1 = " + checkSum + " " + check);
				return true;
			}
			;

		}
		return false;
	}

	/**
	 * Pi >= 0
	 * 
	 * @param p
	 * @return true if not satisfy
	 */
	static boolean constraintF2(PSOParticle p) {
		PSOVector position = p.getPosition();
		for (int i = 0; i < position.getVectorCoordinate().length; i++) {
			if (position.getById(i) < 0)
				System.out.println("F2 at: " + p.getName() + " " + p.getPosition().getVectorRatio().toStringOutput());
			return true;
		}
		return false;
	}

	/**
	 * pi*W <= (W + sum(N)) / n
	 * 
	 * @param cWL
	 * @param mapRTable
	 * @param worker
	 * @param rtable
	 * 
	 * @param p
	 * @return true if not satisfy
	 */

	// static PSOVector constraintF5(int workLoad, PSOVector time, String pid,
	// List<RTable> rtable, PSOVector ratio,
	// HashMap<Integer, List<RTable>> mapRTable, int testCase, PSOParticleRSU p) {
	// int j = 0;
	// double check = 9999;
	// for (RTable r : rtable) {
	// double pai = ratio.getById(j);
	// double lambWL = r.getResource() / r.getReq().getWL();

	// double pen = pai - lambWL;

	// if (!r.getDes().equals(r.getReq().getSrcNode().getName())) { // khong xet req
	// node trong dieu kien nay
	// // pen = pai - lambWL;
	// if (pen > 0) {
	// pen = INFINITY;
	// time.setById(j, time.getById(j) + pen);
	// check = j;
	// }
	// }
	// // addPen += pen;
	// j++;

	// }

	// if (check != 9999) {
	// LogPSO log = LogPSO.getInstance();
	// log.log("\tF3." + j + "\t");
	// // log.log("\n" +worker.toStringOutput() + "\n");
	// }

	// for (int i = 0; i < time.getDim(); i++) {
	// // postion.setById(i, postion.getById(i)+ addPen);
	// }

	// return time;
	// }
}
