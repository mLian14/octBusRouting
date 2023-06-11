package buildVars;

import grb.GurobiExecutor;
import grb.GurobiVariable;
import gurobi.GRB;
import shapeVar.RectFirstVirtualPointVar;
import shapeVar.RectVirtualPointVar;
import shapes.Obstacle;
import shapes.PseudoBase;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

/**
 * @auther lianmeng
 * @create 10.06.23
 */
public class BuildVarsForRectRouting extends BuildVars {

    public ArrayList<RectVirtualPointVar> rectVirtualPointVars;


    public BuildVarsForRectRouting(ArrayList<RectVirtualPointVar> rectVirtualPointVars, ArrayList<Obstacle> obstacles, ArrayList<PseudoBase> slaves, PseudoBase master, GurobiExecutor executor, int m) {
        super(obstacles, slaves, master, executor, m);
        this.rectVirtualPointVars = rectVirtualPointVars;
    }

    public void buildVars(){
        GurobiVariable q;

        /*
        find the boundary of the board
         */
        ArrayList<Integer> xs = new ArrayList<>();
        ArrayList<Integer> ys = new ArrayList<>();
        for (Obstacle o : obstacles) {
            xs.add(o.getMinX());
            xs.add(o.getMaxX());
            ys.add(o.getMinY());
            ys.add(o.getMaxY());
        }
        for (PseudoBase sv : slaves) {
            xs.add(sv.getX());
            ys.add(sv.getY());
        }
        xs.add(master.getX());
        ys.add(master.getY());

        int lb_x = Collections.min(xs);
        int ub_x = Collections.max(xs);
        int lb_y = Collections.min(ys);
        int ub_y = Collections.max(ys);

        for (int i = 0; i < slaves.size(); ++i){
            if (i == 0){
                RectFirstVirtualPointVar vF = new RectFirstVirtualPointVar();
                rectVirtualPointVars.add(vF);
            }else {
                RectVirtualPointVar vp = new RectVirtualPointVar();
                rectVirtualPointVars.add(vp);
            }
        }

        for (RectVirtualPointVar vp : rectVirtualPointVars){
            int i = rectVirtualPointVars.indexOf(vp);
            /*
            Master Relevant
            VVVV
             */
            if (vp instanceof RectFirstVirtualPointVar){
                RectFirstVirtualPointVar vF = (RectFirstVirtualPointVar) vp;
                String name = "v0";
                vF.vm_detour_q = new GurobiVariable(GRB.BINARY, 0, 1, name + "_vm_detour_q");
                executor.addVariable(vF.vm_detour_q);

                vF.vm_dOut_iq = new GurobiVariable(GRB.INTEGER, 0, M, name + "_vm_dOut_iq");
                executor.addVariable(vF.vm_dOut_iq);

                vF.vm_dIn_iq = new GurobiVariable(GRB.INTEGER, 0, M, name + "_vm_dIn_iq");
                executor.addVariable(vF.vm_dIn_iq);

                vF.vm_dist_iq = new GurobiVariable(GRB.INTEGER, 0, M, name + "_vm_dist_iq");
                executor.addVariable(vF.vm_dist_iq);

                vF.aux_vmDist_iqs = buildAuxManhattanVar(name + "_aux_vmDist_iqs_");

                for (Obstacle o : obstacles){
                    String oName = "m" + i + "_" + o.getName();

                    vF.vm_relObstaclesD_qs.put(o, buildBinaryVar(oName + "_vm_relObstaclesD_qs_", 4));

                    q = new GurobiVariable(GRB.BINARY, 0, 1, oName + "_vm_relObstacles_q");
                    executor.addVariable(q);
                    vF.vm_relObstacles_q.put(o, q);

                    vF.vm_corner_qs.put(o, buildBinaryVar(oName + "_vm_corner_qs_", 2));

                    vF.vm_oCoordinate_iqs.put(o, buildBinaryVar(oName + "_vm_oCoordinate_iqs_", 2));

                    vF.vm_startEndObstacles_qs.put(o, buildBinaryVar(oName + "_vm_startEndObstacles_qs_", 2));

                    vF.aux_vmdOut_iqs.put(o, buildAuxManhattanVar(oName + "_aux_vmdOut_iqs_"));

                    vF.aux_vmdIn_iqs.put(o, buildAuxManhattanVar(oName + "_aux_vmdIn_iqs_"));

                    vF.aux_vmdIn_iqs.put(o, buildAuxManhattanVar(oName + "_aux_vmdIn_iqs"));

                    vF.linAux_vmInOut_qs.put(o, buildBinaryVar(oName + "_linAux_vmInOut_qs_", 2));

                    Map<Obstacle, GurobiVariable> vm_omOnCnn_qMap = new HashMap<>();
                    Map<Obstacle, GurobiVariable> vm_dOmOn_iqMap = new HashMap<>();
                    Map<Obstacle, GurobiVariable[]> aux_vmdOmOn_iqsMap = new HashMap<>();
                    Map<Obstacle, GurobiVariable[]> linAux_vmOmOn_qsMap = new HashMap<>();
                    for (Obstacle on : obstacles){
                        String onName = oName + "->" + on.getName();
                        q = new GurobiVariable(GRB.BINARY, 0, 1, onName + "_vm_omOnCnn_q");
                        executor.addVariable(q);
                        vm_omOnCnn_qMap.put(on, q);

                        q = new GurobiVariable(GRB.INTEGER, 0, M, onName + "_vm_dOmOn_iq");
                        executor.addVariable(q);
                        vm_dOmOn_iqMap.put(on, q);

                        aux_vmdOmOn_iqsMap.put(on, buildAuxManhattanVar(onName + "_aux_vmdOmOn_iqs_"));

                        linAux_vmOmOn_qsMap.put(on, buildBinaryVar(onName + "_linAux_vmOmOn_qs_", 2));

                    }
                    vF.vm_omOnCnn_q.put(o, vm_omOnCnn_qMap);
                    vF.vm_dOmOn_iq.put(o, vm_dOmOn_iqMap);
                    vF.aux_vmdOmOn_iqs.put(o, aux_vmdOmOn_iqsMap);
                    vF.linAux_vmOmOn_qs.put(o, linAux_vmOmOn_qsMap);

                }


            }

            vp.x = new GurobiVariable(GRB.INTEGER, lb_x, ub_x, "x" + i);
//            vp.x = new GurobiVariable(GRB.INTEGER, -652, -652, "x" + i);
            executor.addVariable(vp.x);
            vp.y = new GurobiVariable(GRB.INTEGER, lb_y, ub_y, "y" + i);
//            vp.y = new GurobiVariable(GRB.INTEGER, 1576, 1576, "y" + i);
            executor.addVariable(vp.y);

            /*
            vp->vp: detour trigger:
             */
            String name = "v" + i;
            vp.detour_q = new GurobiVariable(GRB.BINARY, 0, 1, "v" + i + "_detour_q");
            executor.addVariable(vp.detour_q);

            vp.dOut_iq = new GurobiVariable(GRB.INTEGER, 0, M, name + "_dOut_iq");
            executor.addVariable(vp.dOut_iq);

            vp.dIn_iq = new GurobiVariable(GRB.INTEGER, 0, M, name + "_dIn_iq");
            executor.addVariable(vp.dIn_iq);

            vp.dist_iq = new GurobiVariable(GRB.INTEGER, 0, M, name + "_dist_iq");
            executor.addVariable(vp.dist_iq);

            vp.aux_dist_iqs = buildAuxManhattanVar(name + "_aux_dist_iqs_");

            vp.corDist_iq = new GurobiVariable(GRB.INTEGER, 0, M, name + "corDist_iq");
            executor.addVariable(vp.corDist_iq);

            for (Obstacle o : obstacles){
                String oName = "v" + i + "_" + o.getName();

                vp.non_qs.put(o, buildBinaryVar(oName + "_non_qs_", 4));
                vp.relD_qs.put(o, buildBinaryVar(oName + "_relD_qs_", 4));
                vp.relObstaclesD_qs.put(o, buildBinaryVar(oName + "_relObstacles_qs_", 4));

                q = new GurobiVariable(GRB.BINARY, 0, 1, oName + "_relObstacles_q");
                executor.addVariable(q);
                vp.relObstacles_q.put(o, q);

                vp.corner_qs.put(o, buildBinaryVar(oName + "_corner_qs_", 2));
                vp.oCoordinate_iqs.put(o, buildIntVar(lb_x, ub_y, oName + "_oCoordinate_iqs_", 2));
                vp.startEndObstacles_qs.put(o, buildBinaryVar(oName + "_startEndObstacles_qs_", 2));

                vp.aux_dOut_iqs.put(o, buildAuxManhattanVar(oName + "_aux_dOut_iqs_"));
                vp.aux_dIn_iqs.put(o, buildAuxManhattanVar(oName + "_aux_dIn_iqs_"));
                vp.linAux_inOut_qs.put(o, buildBinaryVar(oName + "_linAux_inOut_qs_", 2));

                Map<Obstacle, GurobiVariable> omOnCnn_qMap = new HashMap<>();
                Map<Obstacle, GurobiVariable> dOmOn_iqMap = new HashMap<>();
                Map<Obstacle, GurobiVariable[]> aux_dOmOn_iqsMap = new HashMap<>();
                Map<Obstacle, GurobiVariable[]> linAux_OmOn_qsMap = new HashMap<>();
                for (Obstacle on : obstacles){
                    String onName = oName + "->" + on.getName();
                    q = new GurobiVariable(GRB.BINARY, 0, 1, onName + "_omOnCnn_q");
                    executor.addVariable(q);
                    omOnCnn_qMap.put(on, q);

                    q = new GurobiVariable(GRB.INTEGER, 0, M, onName + "_dOmOn_iq");
                    executor.addVariable(q);
                    dOmOn_iqMap.put(on,q);

                    aux_dOmOn_iqsMap.put(on, buildAuxManhattanVar(onName + "_aux_dOmOn_iqs_"));
                    linAux_OmOn_qsMap.put(on, buildBinaryVar(onName + "_linAux_OmOn_qs_", 2));

                }
                vp.omOnCnn_q.put(o, omOnCnn_qMap);
                vp.dOmOn_iq.put(o, dOmOn_iqMap);
                vp.aux_dOmOn_iqs.put(o, aux_dOmOn_iqsMap);
                vp.linAux_OmOn_qs.put(o, linAux_OmOn_qsMap);
            }

            for (PseudoBase sv : slaves){
                String sName = name + "->" + sv.getName();
                q = new GurobiVariable(GRB.BINARY, 0, 1, sName + "_vsCnn_q");
                executor.addVariable(q);
                vp.vsCnn_q.put(sv, q);

                q = new GurobiVariable(GRB.BINARY, 0, 1, sName + "_vs_detour_q");
                executor.addVariable(q);
                vp.vs_detour_q.put(sv, q);

                q = new GurobiVariable(GRB.INTEGER, 0, M, sName + "_vs_dOut_iq");
                executor.addVariable(q);
                vp.vs_dOut_iq.put(sv, q);

                q = new GurobiVariable(GRB.INTEGER, 0, M, sName + "_vs_dIn_iq");
                executor.addVariable(q);
                vp.vs_dIn_iq.put(sv, q);

                q = new GurobiVariable(GRB.INTEGER, 0, M, sName + "_vs_dist_iq");
                executor.addVariable(q);
                vp.vs_dist_iq.put(sv, q);

                vp.aux_vsDist_iqs.put(sv, buildAuxManhattanVar(sName + "_aux_vsDist_iqs_"));

                Map<Obstacle, GurobiVariable[]> vs_relObstaclesD_qsMap = new HashMap<>();
                Map<Obstacle, GurobiVariable> vs_relObstacles_qMap = new HashMap<>();
                Map<Obstacle, GurobiVariable[]> vs_Corner_qsMap = new HashMap<>();
                Map<Obstacle, GurobiVariable[]> vs_oCoordinate_iqsMap = new HashMap<>();
                Map<Obstacle, GurobiVariable[]> vs_startEndObstacles_qsMap = new HashMap<>();
                Map<Obstacle, GurobiVariable[]> aux_vsdOut_iqsMap = new HashMap<>();
                Map<Obstacle, GurobiVariable[]> aux_vsdIn_iqsMap = new HashMap<>();
                Map<Obstacle, GurobiVariable[]> linAux_vsInOut_qsMap = new HashMap<>();

                Map<Obstacle, Map<Obstacle, GurobiVariable>> vs_omOnCnn_qMMap = new HashMap<>();
                Map<Obstacle, Map<Obstacle, GurobiVariable>> vs_dOmOn_iqMMap = new HashMap<>();
                Map<Obstacle, Map<Obstacle, GurobiVariable[]>> aux_vsdOmOn_iqsMMap = new HashMap<>();
                Map<Obstacle, Map<Obstacle, GurobiVariable[]>> linAux_vsOmOn_qsMMap = new HashMap<>();
                for (Obstacle o : obstacles){
                    String soName = sName + "_" + o.getName();

                    vs_relObstaclesD_qsMap.put(o, buildBinaryVar(soName + "_vs_relObstaclesD_qs_", 4));

                    q = new GurobiVariable(GRB.BINARY, 0, 1, soName + "_vs_relObstacles_q");
                    executor.addVariable(q);
                    vs_relObstacles_qMap.put(o, q);

                    vs_Corner_qsMap.put(o, buildBinaryVar(soName + "_vs_Corner_qs_", 2));
                    vs_oCoordinate_iqsMap.put(o, buildIntVar(lb_x, ub_y, soName + "_vs_oCoordinate_iqs_", 2));
                    vs_startEndObstacles_qsMap.put(o, buildBinaryVar(soName + "_vs_startEndObstacles_qs_", 2));

                    aux_vsdOut_iqsMap.put(o, buildAuxManhattanVar(soName + "_aux_vsdOut_iqs_"));
                    aux_vsdIn_iqsMap.put(o, buildAuxManhattanVar(soName + "_aux_vsdIn_iqs_"));

                    linAux_vsInOut_qsMap.put(o, buildBinaryVar(soName + "_linAux_vsInOut_qs_", 2));


                    //om-->on
                    Map<Obstacle, GurobiVariable> vs_omOnCnn_qMap = new HashMap<>();
                    Map<Obstacle, GurobiVariable> vs_dOmOn_iqMap = new HashMap<>();
                    Map<Obstacle, GurobiVariable[]> aux_vsdOmOn_iqsMap = new HashMap<>();
                    Map<Obstacle, GurobiVariable[]> linAux_vsOmOn_qsMap = new HashMap<>();
                    for (Obstacle on : obstacles){
                        String sonName = soName + "->" + on.getName();

                        q = new GurobiVariable(GRB.BINARY, 0, 1, sonName + "_vs_omOnCnn_q");
                        executor.addVariable(q);
                        vs_omOnCnn_qMap.put(on, q);

                        q = new GurobiVariable(GRB.INTEGER, 0, M, sonName + "_vs_dOmOn_iq");
                        executor.addVariable(q);
                        vs_dOmOn_iqMap.put(on, q);

                        aux_vsdOmOn_iqsMap.put(on, buildAuxManhattanVar(sonName + "_aux_vsdOmOn_iqs_"));

                        linAux_vsOmOn_qsMap.put(on, buildBinaryVar(sonName + "_linAux_vsOmOn_qs_", 2));

                    }
                    vs_omOnCnn_qMMap.put(o, vs_omOnCnn_qMap);
                    vs_dOmOn_iqMMap.put(o, vs_dOmOn_iqMap);
                    aux_vsdOmOn_iqsMMap.put(o, aux_vsdOmOn_iqsMap);
                    linAux_vsOmOn_qsMMap.put(o, linAux_vsOmOn_qsMap);

                }
                vp.vs_relObstaclesD_qs.put(sv, vs_relObstaclesD_qsMap);
                vp.vs_relObstacles_q.put(sv, vs_relObstacles_qMap);
                vp.vs_corner_qs.put(sv, vs_Corner_qsMap);
                vp.vs_oCoordinate_iqs.put(sv, vs_oCoordinate_iqsMap);
                vp.vs_startEndObstacles_qs.put(sv, vs_startEndObstacles_qsMap);
                vp.aux_vsdOut_iqs.put(sv, aux_vsdOut_iqsMap);
                vp.aux_vsdIn_iqs.put(sv, aux_vsdIn_iqsMap);
                vp.linAux_vsInOut_qs.put(sv, linAux_vsInOut_qsMap);

                vp.vs_omOnCnn_q.put(sv, vs_omOnCnn_qMMap);
                vp.vs_dOmOn_iq.put(sv, vs_dOmOn_iqMMap);
                vp.aux_vsdOmOn_iqs.put(sv, aux_vsdOmOn_iqsMMap);
                vp.linAux_vsOmOn_qs.put(sv, linAux_vsOmOn_qsMMap);
            }


        }


    }


}
