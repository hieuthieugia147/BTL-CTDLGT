package multihop;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Random;
import java.util.Set;
import java.util.Vector;
import java.util.stream.Collector;

import javax.swing.filechooser.FileNameExtensionFilter;

import PSOSim.PSOSwarm;
import PSOSim.PSOVector;
import multihop.Constants.TYPE;
import multihop.node.NodeBase;

import multihop.node.NodeVehicle;
import multihop.request.RequestBase;
import multihop.request.RequestVehicle;
import multihop.util.AlgUtils;
import multihop.util.TopoUtils;
import multihop.util.TrafficUtils;

public class MainSim {

	static int TS = Constants.TS; // =1

	@SuppressWarnings("unchecked")
	public static void main(String[] args) throws IOException {

		/**
		 * ------------------------------- Prams-------------------------------
		 **/
		boolean DEBUG = true;
		boolean single;
		int hc = 2; // hop-count is number of path to a node

		/**
		 * --- 1.Create topology---
		 */

		// vehicle node
		int _m = 5, _n = 5;
		List<NodeVehicle> topo = new ArrayList<NodeVehicle>();
		topo = (List<NodeVehicle>) TopoUtils.createTopo(_m, _n, 10, Constants.TYPE.VEHICLE.ordinal());
		TopoUtils.updateTimeTopo(topo); // adding moving by time for Vehicle

		//
		/**
		 * --- 2. Create N requests --- id workload request node time_start=time_arrival
		 */
		Queue<RequestBase> reqPiority = new PriorityQueue<RequestBase>(); // store req by time and id
		reqPiority = TrafficUtils.createReqList(topo);
		TopoUtils.setupTopo(topo);

		// transfer queue to list (cz index in queue isn't right): sắp xếp theo thời
		// gian -- file ff_module
		List<RequestBase> req = new ArrayList<RequestBase>();
		while (reqPiority.size() != 0) {
			req.add(reqPiority.poll());
		}

		/**
		 * Loop ts: 0 -- ts_1 -- ts_1+TS
		 */
		final int nTS = Constants.TSIM - 1; // nTS = 14

		FileWriter fListReq; // list requests in each timeslot
		fListReq = new FileWriter("listREQ.txt");
		fListReq.write("\nTS=" + TS + " | nTS=" + nTS);
		fListReq.write("\nTS\tListReqs" + "\n");
		int testCase = 0;

		int[] testCaseList = { 1 }; // {1, 2, 6, 7 }
		for (Integer test : testCaseList) {
			testCase = test;
			for (int h = 1; h <= 1; h++) { // hopcount = 1 or 2
				hc = h; // hc = 1
				int opts = h == 1 ? 1 : 2; // hc=1 -> 1 opts; hc=2 -> 2 opts
				// opts =1;
				for (int s = 0; s < opts; s++) {
					single = s == 0 ? true : false; // single and multi opts

					// clear node
					for (NodeVehicle n : topo) { // ------------------
						n.getqReq().clear();
						n.getDoneReq().clear();
						n.setcWL(0);
						n.setaWL(0);
						// n.setpWL(0);
					}

					// single =false;

					String o = single == true ? "s" : "m";

					FileWriter myWriterPSO;
					myWriterPSO = new FileWriter("topoPSO-" + hc + "." + o + "." + testCase + ".csv");

					FileWriter myWriterPSOserv;
					myWriterPSOserv = new FileWriter("topoPSO_tserv-" + hc + "." + o + "." + testCase + ".csv");

					myWriterPSOserv.write("\n\n" + testCase + "." + hc + "." + single + "\n");
					myWriterPSO.write("\n" + testCase + "," + hc + "," + single + "," + "\n");

					fListReq.write("\n" + testCase + "." + hc + "." + single + "\n");
					myWriterPSO.write("Test case" + "," + "TSIM" + "," + "Workload" + "," + "\n");
					myWriterPSO.write(testCase + "," + Constants.TSIM + "," + "Random" + "," + "\n");
					myWriterPSOserv.write("Test case" + "," + "TSIM" + "," + "Workload" + "," + "\n");
					myWriterPSOserv.write(testCase + "," + Constants.TSIM + "," + "Random" + "," + "\n");

					for (int t = 1; t <= nTS; t++) {
						System.out.println("\nts= " + t
								+ " ---------------------------------------------------------------------------------------------");
						double ts = TS * t;

						// print - begin ts = 1 to 14

						/**
						 * --- 1. Create routing table---
						 */

						HashMap<Integer, List<RTable>> mapRTable = new HashMap<Integer, List<RTable>>();
						List<RTable> rtable = new ArrayList<RTable>();

						// 1.1 prepare request to node in: ts_k < start < ts_k+1
						debug("List REQs: ( prepare req to node )\n ", DEBUG);
						List<NodeVehicle> listNodeReq = new ArrayList<NodeVehicle>(); // node having reqs in ts
						Queue<RequestBase> reqTS = new PriorityQueue<RequestBase>(); // reqTS having in ts

						for (RequestBase r : req) {
							double start = r.getTimeInit();
							if ((start < ts) && (start >= (ts - TS))) {
								listNodeReq.add(r.getSrcNode());
								reqTS.add(r);
							}
						}

						// 1.2 updated create routing-table:rtable and routing-table-with-id:mapRTable
						// with requests
						for (RequestBase r : reqTS) {
							List<RTable> rtableREQ = new ArrayList<RTable>(); // rtable of a request
							int reqId = r.getId();
							// NodeVehicle reqNode = r.getSrcNode();
							// double WL = r.getWL();
							rtableREQ = TopoUtils.createRoutingTable(topo, rtableREQ, r, listNodeReq, hc, single, t);
							rtable.addAll(rtableREQ); // merge all reqs
							mapRTable.put(reqId, rtableREQ); // merge reqs with id
						}
						//
						for (int i = 0; i < rtable.size() - 1; i++) {
							for (int j = i + 1; j < rtable.size() - 1; j++) {
								if (rtable.get(i).getDes().equals(rtable.get(j).getDes())) {

									if (!rtable.get(i).getDes().equals(rtable.get(i).getRoute())) {
										if (!rtable.get(j).getDes().equals(rtable.get(j).getRoute())) {
											int in = rtable.get(i).getsameDes() + 1;
											int im = rtable.get(j).getsameDes() + 1;
											rtable.get(i).setsameDes(in);
											rtable.get(j).setsameDes(im);
										}
									}

								}

							}
						}

						for (int i = 0; i < rtable.size() - 1; i++) {
							System.out.println("\n____________________" + rtable.get(i).getDes()
									+ "________________________" + rtable.get(i).getRoute() + "________________________"
									+ "________________________" + rtable.get(i).getsameDes());
						}

						// TODO: create routing-table for RSU

						/**
						 * --- 2. Run PSO ---
						 */

						// if having req in queue

						if (listNodeReq.size() != 0) {
							// log list_reqs
							fListReq.write(ts + " \t");
							for (RequestBase r : reqTS) {
								fListReq.write(r.getId() + "\t");
							}

							System.out.println("\n***********PSO Running***********\n");

							HashMap<Integer, Double> resultPSO = AlgUtils.getPSO(rtable, mapRTable, testCase, ts);

							Set<Integer> rID = resultPSO.keySet();
							for (Integer id : rID) {
								rtable.get(id).setRatio(resultPSO.get(id));
							}

							/**
							 * --- 3. Logging ---
							 */
							calcTSerPSO(rtable, testCase);

						}
						/**
						 * --- 4. Process CWL and queue in node ---
						 */
						// 4.1 assigned workload in node (all cWL) and adding queue
						insertQ(topo, rtable, t);

						// 4.2 update queue
						updateQ(topo, ts);

						// 4.2 calculate cWL
						updateCWL(topo, ts);

					} // endts

					System.out.println("\n----- DONE REQ ------");
					myWriterPSO.write("Vehicle TOPO\n");
					myWriterPSO.write("\nReqID,\t" + "a(SrcNode),\t" + "i(DesNode),\t" + "k(Path),\t"
							+ "Assign Work load,\t" + "p(Ratio),\t"
							+ "dtTrans,\t" + "tArrival,\t" + "t_wait,\t" + "start,\t" + "t_proc,\t" + "end,\t"
							+ "timeSer(PSO),\t"
							+ "t_serv(layer),\t" + "\n");

					// log for vehicle topo
					for (RequestBase r : req) {
						// double endM=0;
						for (NodeVehicle n : topo) {
							for (RequestBase d : n.getDoneReq()) {
								if (d.getId() == r.getId()) {
									// System.out.println("REQ: " + r.getId() + "\n" + "->" + n.getName() + "_" +
									// d.getRoute() + ": " + d.getStart() + "\t" + d.getEnd());
									RequestVehicle dv = (RequestVehicle) d;
									double t_wait = dv.getStart() - dv.getTimeArrival();
									double t_proc = dv.getEnd() - dv.getStart();
									double t_serv = dv.getTimeTrans() + t_wait + t_proc;
									myWriterPSO.write(r.getId() + "," + dv.getSrcNode().getName() + ","
											+ n.getName()
											+ "," + dv.getRoute() + "," + dv.getWL() + "," + dv.getRatio() + ","
											+ dv.getTimeTrans()
											+ "," + dv.getTimeArrival() + "," + t_wait + "," + dv.getStart()
											+ ","
											+ t_proc + "," + dv.getEnd()
											+ "," + dv.getTimeSer() + "," + t_serv
											+ "," + "\n");
								}
							}
						}
					}

					double wait = 0;
					int count = 0;

					myWriterPSOserv
							.write("\n Vehicle logging\n" + "ID," + "Workload," + "Tser_All(PSO)," + "Tser_(Only)"
									+ "\n");
					for (RequestBase r : req) {
						double endM = 0;
						for (NodeVehicle n : topo) {
							for (RequestBase d : n.getDoneReq()) {
								if (d.getId() == r.getId()) {
									endM = endM > ((RequestVehicle) d).getEnd() ? endM : ((RequestVehicle) d).getEnd();
									wait += (((RequestVehicle) d).getStart() - ((RequestVehicle) d).getTimeArrival());
									count++;
								}
							}
						}

						// endM -= Math.floor(((RequestVehicle) r).getTimeArrival());
						endM -= Math.floor(r.getTimeInit());
						if (endM < 0) {
							endM = 0;
						}

						// endM -= Math.floor(((RequestVehicle) r).getTimeArrival());
						myWriterPSOserv.write(r.getId() + "," + r.getWL() + "," + endM + "," + r.getWL() / 30 + ",\n");

						// System.out.println("AVG: " + wait / count);

					} // end for each case
					myWriterPSOserv.close();
					myWriterPSO.close();
				}
			}

			System.out.println("----FINISH-----");
			fListReq.close();

			// listCWL.close();
			// myWriterPSOserv.close();
			// myWriterPSO.close();
			for (int t = 1; t <= nTS; t++) { // 1-14

				// print - begin ts = 1 to 14

				FileWriter topo_vehicle = new FileWriter("req//topo_vehicle//topo_vehicle" + t + ".csv");
				topo_vehicle.write("Name Position Velocity in" + t + "\n");
				topo_vehicle.write("id,\tname,\tx,\ty,\tvelo,\n");
				for (NodeVehicle nnn : topo) {
					topo_vehicle.write(nnn.getId() + "," + nnn.getName() + "," + nnn.getX()[t] + ","
							+ nnn.getY()[t] + "," + nnn.getVelo()[t] + "\n");
				}
				topo_vehicle.close();
			}
		}

	}

	private static void updateCWL(List<NodeVehicle> topo, double ts) {
		// System.out.println("\n----- Current WL: ");
		// listCWL.write("p");
		for (NodeVehicle n : topo) {
			double pWL = 0; // processed workload
			// double aWL = 0; // all assigned worload
			if ((n.getqReq().size() != 0) || n.getDoneReq().size() != 0) { // node in processing
				// System.out.println("Node " + n.getName());
				// n.getDoneReq().forEach((d) -> {
				// System.out.println("Done req: " + d.getStart() + " " + d.getEnd());
				// });
				// n.getqReq().forEach((q) -> System.out.println("Queue req: " + q.getStart() +
				// " " + q.getEnd()));

				for (RequestBase d : n.getDoneReq()) {
					pWL += (((RequestVehicle) d).getEnd() - ((RequestVehicle) d).getStart()) * n.getRes();
				}

				if (n.getqReq().peek() != null) {
					double lastStart = ((RequestVehicle) n.getqReq().peek()).getStart();
					if (lastStart < ts) {
						pWL += (ts - lastStart) * n.getRes();
					}
				}
				// System.out.print("Process WL: " + pWL + " / " + n.getaWL());
				n.setcWL((n.getaWL() - pWL) < 0 ? 0 : (n.getaWL() - pWL));
				// System.out.println("\tcWL: " + n.getcWL());
			}
			// listCWL.write(pWL + "\t");

			// if (listNodeReq.contains(n)) {
			// listCWL.write("\n" + t + "\t" + n.getId() + "\t" + n.getcWL());
			// if (n.getcWL()>0) {n.setcWL(0);}
			// }
		}
	}

	private static void updateQ(List<NodeVehicle> topo, double ts) {
		// System.out.println("\nUPDATE QUEUE");
		for (NodeVehicle n : topo) {
			boolean check = true;
			while (check && (n.getqReq().peek() != null)) { // còn queue
				RequestVehicle rv = (RequestVehicle) n.getqReq().peek();
				double start1 = rv.getStart();
				double end1 = rv.getEnd();
				// System.out.print("Node: " + n.getName());
				// System.out.println(" REQ: " + start1 + " -> " + end1);
				check = false;
				if (end1 < ts) {
					n.getqReq().peek().setDone(true);
					n.getDoneReq().add(n.getqReq().peek()); // adding to done req
					// System.out.println("doneREQ: " + n.getqReq().peek().getStart() + " -> " +
					// end1);
					n.getqReq().remove(); // req is done, removing
					RequestVehicle nextReq = (RequestVehicle) n.getqReq().peek(); // update next request if data sent
					if (nextReq != null) {
						if (end1 > nextReq.getStart()) {
							// System.out.println("update next req start at: " + end1);
							((RequestVehicle) n.getqReq().peek()).setStart(end1); // start after === end before
							((RequestVehicle) n.getqReq().peek())
									.setEnd(end1 + ((RequestVehicle) n.getqReq().peek()).getTimeProcess());
							check = true;
						} else if (end1 < nextReq.getStart() && nextReq.getStart() < ts) {
							check = true;
						}
					}
				}
			}
		}
	}

	private static void insertQ(List<NodeVehicle> topo, List<RTable> rtable, int t) {
		// System.out.println("\nCALC assigned new workload and ADD new reqs to queue");

		for (NodeVehicle n : topo) {
			double aWL = 0; // all assigned workload as new-workload
			boolean check = true;
			for (RTable r : rtable) {
				double move_data = 0;
				if (r.getDes().equals(n.getName())) { // nếu req des == nodeVehicle name
					aWL += r.getRatio() * r.getReq().getWL(); // PSO chia WL được gửi đến
					// t_process: thời gian xử lý WL
					double t_process = r.getRatio() * r.getReq().getWL() / r.getResource();

					// calc moving WL from srcNode
					if (r.getDes().equals(r.getReq().getSrcNode().getName())) { // nếu req des == source request node
						move_data = aWL - Constants.RES[Constants.TYPE.VEHICLE.ordinal()];
						// total wl in a timeslot <= capacity
						if (move_data > 0) {
							t_process = 1;
							aWL = Constants.RES[Constants.TYPE.VEHICLE.ordinal()];
						} else {
							move_data = 0;
						}
					}

					// adding queue - the arrival task to Node, PSO at ts
					double start = r.getTimeTrans() + (t - 1) * TS;
					NodeVehicle srcNode = r.getReq().getSrcNode();
					RequestBase rq = r.getReq();

					RequestVehicle rv = new RequestVehicle(rq.getId(), rq.getWL(), rq.getSrcNode(), rq.getTimeInit(),
							rq.isDone(), n.getId(), r.getRoute(), start, t_process, r.getRatio(), r.getTimeTrans(),
							start, (start + t_process + r.getTimeTrans()), r.getTimeSer(), move_data);
					// timeArrival == start end = start + t_process

					n.getqReq().add(rv); // nhận req đến hàng đợi

					// moving WL to nodeRSU
				}
			}
			n.setaWL(n.getaWL() + aWL);
		}
	}

	private static void calcTSerPSO(List<RTable> rtable, int testCase) {
		// 3.1 t_ser based PSO in rtable
		for (RTable r : rtable) {
			double compute = 0;
			double trans = 0;
			double workLoad = r.getReq().getWL();
			double subWL = r.getRatio() * workLoad; // new WL
			double totalWL = subWL + r.getcWL(); // adding cWL
			compute = totalWL / r.getResource(); // totalTime

			if (testCase != 2) {
				// calc t_process for all paths to node ~ including t_wait
				for (RTable r2 : rtable) {
					if (r2.getDes().equals(r.getDes())
							&& (r2.getId() != r.getId() || (r2.getReq().getId() != r.getReq().getId()))) {
						// adding route 2hop-2path
						compute += r2.getRatio() * r2.getReq().getWL() / r2.getResource();
						subWL += r2.getRatio() * r2.getReq().getWL(); // adding newWL route2
					}
				}
			}

			trans = (r.getRatio() * workLoad / Constants.BW1[0]) * r.getHop();

			if (r.getId() == 0) {
				trans = 0;
			}
			;

			r.setTimeCompute(compute);
			r.setTimeTrans(trans);
			double ser = compute + trans;
			r.setTimeSer(ser);

		} // END 3.1: LOG TIME IN RTABLE

	}

	private static void debug(String s, boolean mode) {
		if (mode)
			System.out.println(s);
	}

}