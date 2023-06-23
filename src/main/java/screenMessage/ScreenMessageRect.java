package screenMessage;

import grb.GurobiVariable;
import gurobi.GRBException;
import parser.OutputDocument;
import shapeVar.RectFirstVirtualPointVar;
import shapeVar.RectVirtualPointVar;
import shapes.Obstacle;
import shapes.PseudoBase;

import java.util.ArrayList;
import java.util.Map;

/**
 * @auther lianmeng
 * @create 11.06.23
 */
public class ScreenMessageRect extends RetrieveGurobi{
    public ArrayList<Obstacle> obstacles;
    public ArrayList<RectVirtualPointVar> rectVirtualPointVars;
    public ArrayList<PseudoBase> slaves;
    public PseudoBase master;
    public GurobiVariable busLength, branchLength;

    public ScreenMessageRect(OutputDocument output, ArrayList<Obstacle> obstacles, ArrayList<RectVirtualPointVar> rectVirtualPointVars, ArrayList<PseudoBase> slaves, PseudoBase master, GurobiVariable busLength, GurobiVariable branchLength) {
        super(output);
        this.obstacles = obstacles;
        this.rectVirtualPointVars = rectVirtualPointVars;
        this.slaves = slaves;
        this.master = master;
        this.busLength = busLength;
        this.branchLength = branchLength;
    }


    public void showResult() throws GRBException {

        System.out.println("busLength= " + busLength.getIntResult());
        System.out.println("branchLength= " + branchLength.getIntResult());

        ArrayList<PseudoBase> orderedSlaves = new ArrayList<>();
        ArrayList<PseudoBase> virtualPoints = new ArrayList<>();
        for (RectVirtualPointVar vp : rectVirtualPointVars){
            int i = rectVirtualPointVars.indexOf(vp);
            System.out.println("v" + i + " (" + vp.x.getIntResult() + ", " + vp.y.getIntResult() + ")");
            String name = "v" + i;
            //virtualPoints.add(new PseudoBase(vp.x.getIntResult(), vp.y.getIntResult(), name));
            if (vp instanceof RectFirstVirtualPointVar){
                RectFirstVirtualPointVar vF = (RectFirstVirtualPointVar) vp;
                String vmName = name + "->" + master.getName() + "(" + master.getX() + ", " + master.getY() + ")";
                String beCnnName = master.getName() + "(" + master.getX() + ", " + master.getY() + ")";
                retrieveDetour(name, vmName, beCnnName, vF.vm_dist_iq, vF.vm_detour_q, vF.vm_relObstacles_q, vF.vm_corner_qs, vF.vm_oCoordinate_iqs, vF.vm_startEndObstacles_qs, vF.vm_dOut_iq, vF.vm_dIn_iq, vF.vm_omOnCnn_q, vF.vm_dOmOn_iq);
            }
            //vp->vp
            String vvName = name + "->v" + (i+1);
            String beCnnName = "v" + (i+1);
            retrieveDetour(name, vvName, beCnnName, vp.dist_iq, vp.detour_q, vp.relObstacles_q, vp.corner_qs, vp.oCoordinate_iqs, vp.startEndObstacles_qs, vp.dOut_iq, vp.dIn_iq, vp.omOnCnn_q, vp.dOmOn_iq);

            for (PseudoBase sv : slaves){
                if (vp.vsCnn_q.get(sv).getIntResult() == 1){
                    orderedSlaves.add(sv);
                    String vsName = name + "->" + sv.getName() + "(" + sv.getX() + ", " + sv.getY() + ")";
                    beCnnName = sv.getName() + "(" + sv.getX() + ", " + sv.getY() + ")";
                    retrieveDetour(name, vsName, beCnnName, vp.vs_dist_iq.get(sv), vp.vs_detour_q.get(sv), vp.vs_relObstacles_q.get(sv), vp.vs_corner_qs.get(sv), vp.vs_oCoordinate_iqs.get(sv), vp.vs_startEndObstacles_qs.get(sv), vp.vs_dOut_iq.get(sv), vp.vs_dIn_iq.get(sv), vp.vs_omOnCnn_q.get(sv), vp.vs_dOmOn_iq.get(sv));
                }
            }
            System.out.println("=-=-=-=-=-=-=-=-=-==-=-=-=--=-=-=-=-=-=-=-=-=-=-=-==-=-=-=-=-=-=-=-");
        }

        output.setSlaves(orderedSlaves);
        output.setVirtualPoints(virtualPoints);

        //As input for OctilinearFixSlavesRouting
        System.out.println("Obstacles");
        for (Obstacle o : obstacles){
            System.out.println(o.getMinX() + " " + o.getMaxX() + " " + o.getMinY() + " " + o.getMaxY());
        }
        System.out.println("FIN");
        System.out.println("Master");
        System.out.println(master.getName() + " " + master.getX() + " " + master.getY());
        System.out.println("Slave");
        for (PseudoBase sv : orderedSlaves){
            System.out.println(sv.getName() + " " + sv.getX() + " " + sv.getY());
        }
        System.out.println("FIN");
        System.out.println("slaveC 1");
        System.out.println("busC 1");


    }

    private void retrieveDetour(String name, String baseName, String beCnnName, GurobiVariable dist_iq, GurobiVariable detour_q, Map<Obstacle, GurobiVariable> relObstacles_q, Map<Obstacle, GurobiVariable[]> corner_qs, Map<Obstacle, GurobiVariable[]> oCoordinate_iqs, Map<Obstacle, GurobiVariable[]> startEndObstacles_qs, GurobiVariable dOut_iq, GurobiVariable dIn_iq, Map<Obstacle, Map<Obstacle, GurobiVariable>> omOnCnn_q, Map<Obstacle, Map<Obstacle, GurobiVariable>> dOmOn_iq) throws GRBException {

        if (detour_q.getIntResult() == 1){
            System.out.println("==============" + baseName + ": detour:" + dist_iq.getIntResult());
            for (Obstacle o : obstacles){
                if (relObstacles_q.get(o).getIntResult() == 1){
                    System.out.print(o.getName() + ": ");
                    System.out.print("corner= " + convertGrbIntArrayToString(corner_qs.get(o)) + "-");
                    System.out.print("(" + oCoordinate_iqs.get(o)[0].getIntResult() + ", " + oCoordinate_iqs.get(o)[1].getIntResult() + ")");
                    System.out.println();
                    if (startEndObstacles_qs.get(o)[0].getIntResult() == 1){
                        System.out.print(name + "->" + o.getName() + ":");
                        System.out.print(dOut_iq.getIntResult() + "||");
                    }
                    for (Obstacle on : obstacles){
                        if (omOnCnn_q.get(o).get(on).getIntResult() == 1){
                            System.out.print("->" + on.getName() + ":");
                            System.out.print(dOmOn_iq.get(o).get(on).getIntResult() + "||");
                        }
                    }
                    if (startEndObstacles_qs.get(o)[1].getIntResult() == 1){
                        System.out.print(o.getName() + "->" + beCnnName + ":");
                        System.out.print(dIn_iq.getIntResult()  + "||");
                    }
                    System.out.println();

                }
            }
        }else {
            System.out.println(baseName + "No detour!" + dist_iq.getIntResult());
        }
    }
}
