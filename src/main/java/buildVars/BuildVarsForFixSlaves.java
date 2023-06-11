package buildVars;

import grb.GurobiExecutor;
import grb.GurobiVariable;
import gurobi.GRB;
import shapeVar.OctFirstVirtualPointFixSlaveVar;
import shapeVar.OctVirtualPointFixSlaveVar;
import shapes.Obstacle;
import shapes.PseudoBase;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

/**
 * @auther lianmeng
 * @create 08.06.23
 */
public class BuildVarsForFixSlaves extends BuildVars {


    public ArrayList<OctVirtualPointFixSlaveVar> octVirtualPointFixSlaveVars;


    public BuildVarsForFixSlaves(ArrayList<Obstacle> obstacles, ArrayList<OctVirtualPointFixSlaveVar> octVirtualPointFixSlaveVars, ArrayList<PseudoBase> slaves, PseudoBase master, GurobiExecutor executor, int m) {
        super(obstacles, slaves, master, executor, m);
        this.octVirtualPointFixSlaveVars = octVirtualPointFixSlaveVars;
    }

    public void buildVariables(){
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
                OctFirstVirtualPointFixSlaveVar vF = new OctFirstVirtualPointFixSlaveVar(slaves.get(i));
                octVirtualPointFixSlaveVars.add(vF);
            }else {
                OctVirtualPointFixSlaveVar vp = new OctVirtualPointFixSlaveVar(slaves.get(i));
                octVirtualPointFixSlaveVars.add(vp);
            }
        }
        for (OctVirtualPointFixSlaveVar vp : octVirtualPointFixSlaveVars){
            int i = octVirtualPointFixSlaveVars.indexOf(vp);
            /*
            Master Relevant
            VVVV
             */
            if (vp instanceof OctFirstVirtualPointFixSlaveVar){
                OctFirstVirtualPointFixSlaveVar vF = (OctFirstVirtualPointFixSlaveVar) vp;

                vF.vm_detour_q = new GurobiVariable(GRB.BINARY, 0, 1, "v" + i + "_vm_detour_q");
                executor.addVariable(vF.vm_detour_q);
                /*
                vm_dOut_cqs
                0: d_vi->:min
                1: d_vi->:xy
                 */
                vF.vm_dOut_cqs = buildIntVar(0, M, "v" + i + "_vm_dOut_cqs_", 2);
                /*
                vm_dIn_cqs
                0: d->vj:min
                1: d->vj:xy
                 */
                vF.vm_dIn_cqs = buildIntVar(0, M, "v" + i + "_vm_dIn_cqs_", 2);
                /*
                vm_dist_cqs:
                0: dv0->master:min
                1: dv0->master:xy
                 */
                vF.vm_dist_cqs = buildIntVar(0, M, "v" + i + "_vm_dist_cqs_", 2);
                /*
                Auxiliary absolute values:
                For pathLength v0->master without detour
                 */
                vF.aux_vm_dist_iqs = buildAuxIntVar("v" + i + "_aux_vm_dist_iqs_");
                vF.auxQ_vm_dist = new GurobiVariable(GRB.BINARY, 0, 1, "v" + i + "_auxQ_vm_dist");
                executor.addVariable(vF.auxQ_vm_dist);
                for (Obstacle o : obstacles){
                    String name = "m" + i + "_" + o.getName();
                    /*
                    vm_relObstacles_qs
                     */
                    vF.vm_relObstacles_qs.put(o, buildBinaryVar(name + "_vm_relObstacles_qs_", 4));
                    /*
                    vm_relObstaclesD_qs
                     */
                    vF.vm_relObstaclesD_qs.put(o, buildBinaryVar(name + "_vm_relObstaclesD_qs_", 4));
                    /*
                    v0->master: indicate relevant Obstacles
                     */
                    q = new GurobiVariable(GRB.BINARY, 0, 1, name + "_vm_relObstacles_q");
                    executor.addVariable(q);
                    vF.vm_relObstacles_q.put(o, q);
                    /*
                    vm_enterCorner_qs, vm_leaveCorner_qs
                     */
                    vF.vm_enterCorner_qs.put(o, buildBinaryVar(name + "_vm_enterCorner_qs_", 4));
                    vF.vm_leaveCorner_qs.put(o, buildBinaryVar(name + "_vm_leaveCorner_qs_", 4));


                    /*
                    vm_enterCoordinate_iqs, vm_leaveCoordinate_iqs
                    0: x_m
                    1: y_m
                     */
                    vF.vm_enterCoordinate_iqs.put(o, buildBinaryVar(name + "_vm_enterCoordinate_iqs_", 2));
                    vF.vm_leaveCoordinate_iqs.put(o, buildBinaryVar(name + "_vm_leaveCoordinate_iqs_", 2));
                    /*
                    Auxiliary absolute values (Manhattan): aux_vmdOm_iqs
                    Manhattan distance between enter and leave corners
                     */
                    vF.aux_vmdOm_iqs.put(o, buildAuxManhattanVar(name + "_aux_vmdOm_iqs_"));
                    /*
                    vm_startEndObstacle_qs
                    0: vi->
                    1: ->vi+1
                     */
                    vF.vm_startEndObstacles_qs.put(o, buildBinaryVar(name + "_vm_startEndObstacles_qs_", 2));

                    /*
                    Auxiliary absolute values: aux_vsdOut_iqs
                    For pathLength.stpTo
                     */
                    vF.aux_vmdOut_iqs.put(o, buildAuxIntVar(name + "aux_vmdOut_iqs_"));
                    q = new GurobiVariable(GRB.BINARY, 0, 1, name + "_auxQ_vmdOut");
                    executor.addVariable(q);
                    vF.auxQ_vmdOut.put(o, q);
                    /*
                    Auxiliary absolute values: aux_vsdIn_iqs
                    For pathLength.toEsp
                     */
                    vF.aux_vmdIn_iqs.put(o, buildAuxIntVar(name + "_aux_vmdIn_iqs"));
                    q = new GurobiVariable(GRB.BINARY, 0, 1, name + "_auxQ_vmdIn");
                    executor.addVariable(q);
                    vF.auxQ_vmdIn.put(o,q);
                    /*
                    vp->slave: linearize Product
                    q_->m * om.relO_q (cnnRules.5)
                    q_m-> * om.relO_q (cnnRules.6)
                     */
                    vF.linAux_vmInOut_qs.put(o, buildBinaryVar(name + "_linAux_vmInOut_qs", 2));

                    /*
                    vm_omOnCnn_q
                     */
                    Map<Obstacle, GurobiVariable> vm_omOnCnn_qMap = new HashMap<>();
                    /*
                    vm_dOmOn_cqs
                     */
                    Map<Obstacle, GurobiVariable[]> vm_dOmOn_cqsMap = new HashMap<>();
                    /*
                    Auxiliary absolute values: aux_vmdOmOn_iqs
                    For pathLength.mTon
                     */
                    Map<Obstacle, GurobiVariable[]> aux_vmdOmOn_iqsMap = new HashMap<>();
                    Map<Obstacle, GurobiVariable> auxQ_vmdOmOnMap = new HashMap<>();
                    /*
                    linearize Product
                    q_m->n * on.relO_q (cnnRules.3)
                    q_m->n * om.relO_q (cnnRules.4)
                     */
                    Map<Obstacle, GurobiVariable[]> linAux_vmOmOn_qsMap = new HashMap<>();
                    for (Obstacle on : obstacles){
                        String onName = name + "->" + on.getName();
                        q = new GurobiVariable(GRB.BINARY, 0, 1, onName + "_vm_omOnCnn_q");
                        executor.addVariable(q);
                        vm_omOnCnn_qMap.put(on, q);
                        vm_dOmOn_cqsMap.put(on, buildIntVar(0, M, onName + "_vm_dOmOn_cqs", 2));

                        linAux_vmOmOn_qsMap.put(on, buildBinaryVar(onName + "_linAux_vmOmOn_qs_", 2));

                        aux_vmdOmOn_iqsMap.put(on, buildAuxIntVar(onName + "_aux_vmdOmOn_iqs_"));
                        q = new GurobiVariable(GRB.BINARY, 0, 1, onName + "_auxQ_vmdOmOn");
                        executor.addVariable(q);
                        auxQ_vmdOmOnMap.put(on, q);

                    }
                    vF.vm_omOnCnn_q.put(o, vm_omOnCnn_qMap);
                    vF.vm_dOmOn_cqs.put(o, vm_dOmOn_cqsMap);
                    vF.linAux_vmOmOn_qs.put(o, linAux_vmOmOn_qsMap);
                    vF.aux_vmdOmOn_iqs.put(o, aux_vmdOmOn_iqsMap);
                    vF.auxQ_vmdOmOn.put(o, auxQ_vmdOmOnMap);

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
            vp.detour_q = new GurobiVariable(GRB.BINARY, 0, 1, "v" + i + "_detour_q");
            executor.addVariable(vp.detour_q);

            /*
            dOut_cqs
            0: d_vi->:min
            1: d_vi->:xy
             */
            vp.dOut_cqs = buildIntVar(0, M, "v" + i + "_dOut_cqs_", 2);
            /*
            dIn_cqs
            0: d->vj:min
            1: d->vj:xy
             */
            vp.dIn_cqs = buildIntVar(0, M, "v" + i + "_dIn_cqs_", 2);
            /*
            dist_cqs: vv_dist
            0: dvp->vp:min
            1: dvp->vp:xy
             */
            vp.dist_cqs = buildIntVar(0, M, "v" + i + "_dist_cqs_", 2);
            /*
            Auxiliary absolute values:
            For pathLength vp->vp without detour
             */
            vp.aux_dist_iqs = buildAuxIntVar("v" + i + "_aux_dist_iqs_");
            vp.auxQ_dist = new GurobiVariable(GRB.BINARY, 0, 1, "v" + i + "_auxQ_dist");
            executor.addVariable(vp.auxQ_dist);

            for (Obstacle o : obstacles){
                String oName = "v" + i + "_" + o.getName();
                /*
                Non-overlapping
                 */
                vp.non_qs.put(o, buildBinaryVar(oName + "_non_qs_", 4));
                /*
                dir_qs
                 */
                vp.dir_qs.put(o, buildBinaryVar(oName + "_dir_qs_", 4));
                /*
                rel_qs
                 */
                vp.rel_qs.put(o, buildBinaryVar(oName + "_rel_qs_", 4));
                /*
                0: nonL * nonT
                1: nonR * nonT
                2: nonL * nonB
                3: nonR * nonB
                 */
                vp.auxRel_qs.put(o, buildBinaryVar(oName + "_auxRel_qs_", 4));
                /*
                relD_qs
                 */
                vp.relD_qs.put(o, buildBinaryVar(oName + "_relD_qs_", 4));
                /*
                relObstacles_qs
                 */
                vp.relObstacles_qs.put(o, buildBinaryVar(oName + "_relObstacles_qs_", 4));
                /*
                relObstaclesD_qs
                 */
                vp.relObstaclesD_qs.put(o, buildBinaryVar(oName + "_relObstacles_qs_", 4));
                /*
                indicate the relevance of each obstacle
                 */
                q = new GurobiVariable(GRB.BINARY, 0, 1, oName + "_relObstacles_q");
                executor.addVariable(q);
                vp.relObstacles_q.put(o, q);
                /*
                enterCorner_qs, leaveCorner_qs
                 */
                vp.enterCorner_qs.put(o, buildBinaryVar(oName + "_enterCorner_qs_", 4));
                vp.leaveCorner_qs.put(o, buildBinaryVar(oName + "_leaveCorner_qs_", 4));

                /*
                enterCoordinate_iqs, leaveCoordinate_iqs
                0: x_m
                1: y_m
                 */
                vp.enterCoordinate_iqs.put(o, buildBinaryVar(oName + "_enterCoordinate_iqs_", 2));
                vp.leaveCoordinate_iqs.put(o, buildBinaryVar(oName + "_leaveCoordinate_iqs_", 2));
                /*
                Auxiliary absolute values (Manhattan): aux_dOm_iqs
                Manhattan distance between enter and leave corners
                 */
                vp.aux_dOm_iqs.put(o, buildAuxManhattanVar(oName + "_aux_dOm_iqs_"));
                /*
                startEndObstacle_qs
                0: vi->
                1: ->vi+1
                 */
                vp.startEndObstacles_qs.put(o, buildBinaryVar(oName + "_startEndObstacles_qs_", 2));

                /*
                Auxiliary absolute values: aux_dOut_iqs
                For pathLength.stpTo
                 */
                vp.aux_dOut_iqs.put(o, buildAuxIntVar(oName + "aux_dOut_iqs_"));
                q = new GurobiVariable(GRB.BINARY, 0, 1, oName + "_auxQ_dOut");
                executor.addVariable(q);
                vp.auxQ_dOut.put(o, q);
                /*
                Auxiliary absolute values: aux_dIn_iqs
                For pathLength.toEvp
                 */
                vp.aux_dIn_iqs.put(o, buildAuxIntVar(oName + "_aux_dIn_iqs"));
                q = new GurobiVariable(GRB.BINARY, 0, 1, oName + "_auxQ_dIn");
                executor.addVariable(q);
                vp.auxQ_dIn.put(o,q);
                /*
                linearize Product
                q_->m * om.relO_q (cnnRules.5)
                q_m-> * om.relO_q (cnnRules.6)
                 */
                vp.linAux_inOut_qs.put(o, buildBinaryVar(oName + "_linAux_inOut_qs", 2));

                /*
                omOnCnn_q
                 */
                Map<Obstacle, GurobiVariable> omOnCnn_qMap = new HashMap<>();
                /*
                dOmOn_cqs
                 */
                Map<Obstacle, GurobiVariable[]> dOmOn_cqsMap = new HashMap<>();
                /*
                Auxiliary absolute values: aux_dOmOn_iqs
                For pathLength.mTon
                 */
                Map<Obstacle, GurobiVariable[]> aux_dOmOn_iqsMap = new HashMap<>();
                Map<Obstacle, GurobiVariable> auxQ_dOmOnMap = new HashMap<>();
                /*
                linearize Product
                q_m->n * on.relO_q (cnnRules.3)
                q_m->n * om.relO_q (cnnRules.4)
                 */
                Map<Obstacle, GurobiVariable[]> linAux_OmOn_qsMap = new HashMap<>();
                for (Obstacle on : obstacles){
                    String nameE = oName + "->" + on.getName();
                    q = new GurobiVariable(GRB.BINARY, 0, 1, nameE + "_omOnCnn_q");
                    executor.addVariable(q);
                    omOnCnn_qMap.put(on, q);
                    dOmOn_cqsMap.put(on, buildIntVar(0, M, nameE + "_dOmOn_cqs", 2));

                    linAux_OmOn_qsMap.put(on, buildBinaryVar(nameE + "_linAux_OmOn_qs_", 2));

                    aux_dOmOn_iqsMap.put(on, buildAuxIntVar(nameE + "_aux_dOmOn_iqs_"));
                    q = new GurobiVariable(GRB.BINARY, 0, 1, nameE + "_auxQ_dOmOn");
                    executor.addVariable(q);
                    auxQ_dOmOnMap.put(on, q);

                }
                vp.omOnCnn_q.put(o, omOnCnn_qMap);
                vp.dOmOn_cqs.put(o, dOmOn_cqsMap);
                vp.linAux_OmOn_qs.put(o, linAux_OmOn_qsMap);
                vp.aux_dOmOn_iqs.put(o, aux_dOmOn_iqsMap);
                vp.auxQ_dOmOn.put(o, auxQ_dOmOnMap);

            }

            /*
            Slave Relevant
            VVVV
             */
            vp.vs_detour_q = new GurobiVariable(GRB.BINARY, 0, 1, "v" + i + "_vs_detour_q");
            executor.addVariable(vp.vs_detour_q);
            /*
            vs_dOut_cqs
            0: d_vi->:min
            1: d_vi->:xy
             */
            vp.vs_dOut_cqs = buildIntVar(0, M, "v" + i + "_vs_dOut_cqs_", 2);
            /*
            vs_dIn_cqs
            0: d->vj:min
            1: d->vj:xy
             */
            vp.vs_dIn_cqs = buildIntVar(0, M, "v" + i + "_vs_dIn_cqs_", 2);
            /*
            vs_dist_cqs:
            0: dvp->slave:min
            1: dvp->slave:xy
             */
            vp.vs_dist_cqs = buildIntVar(0, M, "v" + i + "_vs_dist_cqs_", 2);
            /*
            Auxiliary absolute values:
            For pathLength vp->vp without detour
             */
            vp.aux_vs_dist_iqs = buildAuxIntVar("v" + i + "_aux_vs_dist_iqs_");
            vp.auxQ_vs_dist = new GurobiVariable(GRB.BINARY, 0, 1, "v" + i + "_auxQ_vs_dist");
            executor.addVariable(vp.auxQ_vs_dist);
            for (Obstacle o : obstacles){
                String name = "s" + i + "_" + o.getName();
                /*
                vs_relObstacles_qs
                 */
                vp.vs_relObstacles_qs.put(o, buildBinaryVar(name + "_vs_relObstacles_qs_", 4));
                /*
                vs_relObstaclesD_qs
                 */
                vp.vs_relObstaclesD_qs.put(o, buildBinaryVar(name + "_vs_relObstaclesD_qs_", 4));
                /*
                vp->slave: indicate relevant Obstacles
                 */
                q = new GurobiVariable(GRB.BINARY, 0, 1, name + "_vs_relObstacles_q");
                executor.addVariable(q);
                vp.vs_relObstacles_q.put(o, q);
                /*
                vs_enterCorner_qs, vs_leaveCorner_qs
                 */
                vp.vs_enterCorner_qs.put(o, buildBinaryVar(name + "_vs_enterCorner_qs_", 4));
                vp.vs_leaveCorner_qs.put(o, buildBinaryVar(name + "_vs_leaveCorner_qs_", 4));

                /*
                vs_enterCoordinate_iqs, vs_leaveCoordinate_iqs
                0: x_m
                1: y_m
                 */
                vp.vs_enterCoordinate_iqs.put(o, buildBinaryVar(name + "_vs_enterCoordinate_iqs_", 2));
                vp.vs_leaveCoordinate_iqs.put(o, buildBinaryVar(name + "_vs_leaveCoordinate_iqs_", 2));
                /*
                Auxiliary absolute values (Manhattan): aux_vsdOm_iqs
                Manhattan distance between enter and leave corners
                 */
                vp.aux_vsdOm_iqs.put(o, buildAuxManhattanVar(name + "_aux_vsdOm_iqs_"));
                /*
                vs_startEndObstacle_qs
                0: vi->
                1: ->vi+1
                 */
                vp.vs_startEndObstacles_qs.put(o, buildBinaryVar(name + "_vs_startEndObstacles_qs_", 2));

                /*
                Auxiliary absolute values: aux_vsdOut_iqs
                For pathLength.stpTo
                 */
                vp.aux_vsdOut_iqs.put(o, buildAuxIntVar(name + "aux_vsdOut_iqs_"));
                q = new GurobiVariable(GRB.BINARY, 0, 1, name + "_auxQ_vsdOut");
                executor.addVariable(q);
                vp.auxQ_vsdOut.put(o, q);
                /*
                Auxiliary absolute values: aux_vsdIn_iqs
                For pathLength.toEsp
                 */
                vp.aux_vsdIn_iqs.put(o, buildAuxIntVar(name + "_aux_vsdIn_iqs"));
                q = new GurobiVariable(GRB.BINARY, 0, 1, name + "_auxQ_vsdIn");
                executor.addVariable(q);
                vp.auxQ_vsdIn.put(o,q);
                /*
                vp->slave: linearize Product
                q_->m * om.relO_q (cnnRules.5)
                q_m-> * om.relO_q (cnnRules.6)
                 */
                vp.linAux_vsInOut_qs.put(o, buildBinaryVar(name + "_linAux_vsInOut_qs", 2));

                /*
                vs_omOnCnn_q
                 */
                Map<Obstacle, GurobiVariable> vs_omOnCnn_qMap = new HashMap<>();
                /*
                vs_dOmOn_cqs
                 */
                Map<Obstacle, GurobiVariable[]> vs_dOmOn_cqsMap = new HashMap<>();
                /*
                Auxiliary absolute values: aux_vsdOmOn_iqs
                For pathLength.mTon
                 */
                Map<Obstacle, GurobiVariable[]> aux_vsdOmOn_iqsMap = new HashMap<>();
                Map<Obstacle, GurobiVariable> auxQ_vsdOmOnMap = new HashMap<>();
                /*
                linearize Product
                q_m->n * on.relO_q (cnnRules.3)
                q_m->n * om.relO_q (cnnRules.4)
                 */
                Map<Obstacle, GurobiVariable[]> linAux_vsOmOn_qsMap = new HashMap<>();
                for (Obstacle on : obstacles){
                    String nameE = name + "->" + on.getName();
                    q = new GurobiVariable(GRB.BINARY, 0, 1, nameE + "_vs_omOnCnn_q");
                    executor.addVariable(q);
                    vs_omOnCnn_qMap.put(on, q);
                    vs_dOmOn_cqsMap.put(on, buildIntVar(0, M, nameE + "_vs_dOmOn_cqs", 2));

                    linAux_vsOmOn_qsMap.put(on, buildBinaryVar(nameE + "_linAux_vsOmOn_qs_", 2));

                    aux_vsdOmOn_iqsMap.put(on, buildAuxIntVar(nameE + "_aux_vsdOmOn_iqs_"));
                    q = new GurobiVariable(GRB.BINARY, 0, 1, nameE + "_auxQ_vsdOmOn");
                    executor.addVariable(q);
                    auxQ_vsdOmOnMap.put(on, q);

                }
                vp.vs_omOnCnn_q.put(o, vs_omOnCnn_qMap);
                vp.vs_dOmOn_cqs.put(o, vs_dOmOn_cqsMap);
                vp.linAux_vsOmOn_qs.put(o, linAux_vsOmOn_qsMap);
                vp.aux_vsdOmOn_iqs.put(o, aux_vsdOmOn_iqsMap);
                vp.auxQ_vsdOmOn.put(o, auxQ_vsdOmOnMap);

            }

        }


    }



    /**
     * auxiliary variables to compute distance regarding octilinear routing
     * @param varName name
     * @return Array of auxiliary Gurobi variables
     * 0: |x1 - x2|
     * 1: |y1 - y2|
     * 2: min(|x1 - x2|, |y1 - y2|)
     * 3: ||x1 - x2| - |y1 - y2||
     * 4: x1 - x2
     * 5: y1 - y2
     * 6: |x1 - x2| - |y1 - y2|
     */
    protected GurobiVariable[] buildAuxIntVar(String varName) {
        GurobiVariable[] qs = new GurobiVariable[7];
        for (int i = 0; i < 4; ++i) {
            qs[i] = new GurobiVariable(GRB.INTEGER, 0, M, varName + i);
            executor.addVariable(qs[i]);
        }
        for (int i = 4; i < 7; ++i) {
            qs[i] = new GurobiVariable(GRB.INTEGER, -M, M, varName + i);
            executor.addVariable(qs[i]);
        }
        return qs;
    }




}
