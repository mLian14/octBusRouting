package screenMessage;

import grb.GurobiVariable;
import gurobi.GRBException;
import shapeVar.OctFirstVirtualPointFixSlaveVar;
import shapeVar.OctVirtualPointFixSlaveVar;
import shapes.Obstacle;
import shapes.PseudoBase;

import java.util.ArrayList;
import java.util.Map;

/**
 * @auther lianmeng
 * @create 10.06.23
 */
public class ScreenMessageFixSlaves extends RetrieveGurobi {
    public ArrayList<Obstacle> obstacles;
    public ArrayList<OctVirtualPointFixSlaveVar> octVirtualPointFixSlaveVars;
    public PseudoBase master;
    public GurobiVariable busMin, busXY, branchMin, branchXY;

    public ScreenMessageFixSlaves(ArrayList<Obstacle> obstacles, ArrayList<OctVirtualPointFixSlaveVar> octVirtualPointFixSlaveVars, PseudoBase master, GurobiVariable busMin, GurobiVariable busXY, GurobiVariable branchMin, GurobiVariable branchXY) {
        this.obstacles = obstacles;
        this.octVirtualPointFixSlaveVars = octVirtualPointFixSlaveVars;
        this.master = master;
        this.busMin = busMin;
        this.busXY = busXY;
        this.branchMin = branchMin;
        this.branchXY = branchXY;
    }

    public void showResult() throws GRBException {
        System.out.println("busMin= " + busMin.getIntResult());
        System.out.println("busXY= " + busXY.getIntResult());
        System.out.println("branchMin= " + branchMin.getIntResult());
        System.out.println("branchXY= " + branchXY.getIntResult());

        for (OctVirtualPointFixSlaveVar vp : octVirtualPointFixSlaveVars){
            PseudoBase sv = vp.slave;
            int i = octVirtualPointFixSlaveVars.indexOf(vp);
            System.out.println("v" + i + " (" + vp.x.getIntResult() + ", " + vp.y.getIntResult() + ")");
            String name = "v" + i;
            if (vp instanceof OctFirstVirtualPointFixSlaveVar){
               OctFirstVirtualPointFixSlaveVar vF = (OctFirstVirtualPointFixSlaveVar) vp;
                String vmName = name + "->" + master.getName() + "(" + master.getX() + ", " + master.getY() + ")";
                String beCnnName = master.getName() + "(" + master.getX() + ", " + master.getY() + ")";
                retrieveDetour(name, vmName, beCnnName, vF.vm_dist_cqs, vF.vm_detour_q, vF.relObstacles_q, vF.vm_enterCorner_qs, vF.vm_leaveCorner_qs, vF.vm_enterCoordinate_iqs, vF.vm_leaveCoordinate_iqs, vF.vm_startEndObstacles_qs, vF.vm_dOut_cqs, vF.vm_dIn_cqs, vF.vm_omOnCnn_q, vF.vm_dOmOn_cqs);
            }
            //vp->vp
            String vvName = name + "->v" + (i+1);
            String beCnnName = "v" + (i+1);
            retrieveDetour(name, vvName, beCnnName, vp.dist_cqs, vp.detour_q, vp.relObstacles_q, vp.enterCorner_qs, vp.leaveCorner_qs, vp.enterCoordinate_iqs, vp.leaveCoordinate_iqs, vp.startEndObstacles_qs, vp.dOut_cqs, vp.dIn_cqs, vp.omOnCnn_q, vp.dOmOn_cqs);

            String vsName = name + "->" + sv.getName() + "(" + sv.getX() + ", " + sv.getY() + ")";
            beCnnName = sv.getName() + "(" + sv.getX() + ", " + sv.getY() + ")";
            retrieveDetour(name, vsName, beCnnName, vp.vs_dist_cqs, vp.vs_detour_q, vp.vs_relObstacles_q, vp.vs_enterCorner_qs, vp.vs_leaveCorner_qs, vp.vs_enterCoordinate_iqs, vp.vs_leaveCoordinate_iqs, vp.vs_startEndObstacles_qs, vp.vs_dOut_cqs, vp.vs_dIn_cqs, vp.vs_omOnCnn_q, vp.vs_dOmOn_cqs);
            System.out.println("=-=-=-=-=-=-=-=-=-==-=-=-=--=-=-=-=-=-=-=-=-=-=-=-==-=-=-=-=-=-=-=-");
        }




    }

    private void retrieveDetour(String name, String baseName, String beCnnName, GurobiVariable[] dist_cqs, GurobiVariable detour_q, Map<Obstacle, GurobiVariable> relObstacles_q, Map<Obstacle, GurobiVariable[]> enterCorner_qs, Map<Obstacle, GurobiVariable[]> leaveCorner_qs, Map<Obstacle, GurobiVariable[]> enterCoordinate_iqs, Map<Obstacle, GurobiVariable[]> leaveCoordinate_iqs, Map<Obstacle, GurobiVariable[]> startEndObstacles_qs, GurobiVariable[] dOut_cqs, GurobiVariable[] dIn_cqs, Map<Obstacle, Map<Obstacle, GurobiVariable>> omOnCnn_q, Map<Obstacle, Map<Obstacle, GurobiVariable[]>> dOmOn_cqs) throws GRBException {

        if (detour_q.getIntResult() == 1){
            System.out.println("==============" + baseName + ": detour:" + convertGrbIntArrayToString(dist_cqs));
            for (Obstacle o : obstacles){
                if (relObstacles_q.get(o).getIntResult() == 1){
                    System.out.print(o.getName() + ": ");
                    System.out.print("enterCorner= " + convertGrbIntArrayToString(enterCorner_qs.get(o)) + "-");
                    System.out.print("(" + enterCoordinate_iqs.get(o)[0] + ", " + enterCoordinate_iqs.get(o)[1] + ")");
                    System.out.print("||leaveCorner= " + convertGrbIntArrayToString(leaveCorner_qs.get(o)) + "-");
                    System.out.print("(" + leaveCoordinate_iqs.get(o)[0] + "," + leaveCoordinate_iqs.get(o)[1] + ")");
                    System.out.println();
                    if (startEndObstacles_qs.get(o)[0].getIntResult() == 1){
                        System.out.print(name + "->" + o.getName() + ":");
                        System.out.print(convertGrbIntArrayToString(dOut_cqs) + "||");
                    }
                    for (Obstacle on : obstacles){
                        if (omOnCnn_q.get(o).get(on).getIntResult() == 1){
                            System.out.print("->" + on.getName() + ":");
                            System.out.print(convertGrbIntArrayToString(dOmOn_cqs.get(o).get(on)) + "||");
                        }
                    }
                    if (startEndObstacles_qs.get(o)[1].getIntResult() == 1){
                        System.out.print(o.getName() + "->" + beCnnName + ":");
                        System.out.print(convertGrbIntArrayToString(dIn_cqs)  + "||");
                    }
                    System.out.println();

                }
            }
        }else {
            System.out.println(baseName + "No detour!" + convertGrbIntArrayToString(dist_cqs));
        }
    }


}
