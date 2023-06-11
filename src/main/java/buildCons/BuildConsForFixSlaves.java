package buildCons;

import grb.GurobiConstraint;
import grb.GurobiExecutor;
import grb.GurobiVariable;
import gurobi.GRBException;
import shapeVar.OctFirstVirtualPointFixSlaveVar;
import shapeVar.OctVirtualPointFixSlaveVar;
import shapes.Obstacle;
import shapes.PseudoBase;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Map;

/**
 * @auther lianmeng
 * @create 09.06.23
 */
public class BuildConsForFixSlaves extends BuildCons {

    public ArrayList<OctVirtualPointFixSlaveVar> octVirtualPointFixSlaveVars;

    public GurobiVariable busMin, busXY, branchMin, branchXY;



    public BuildConsForFixSlaves(ArrayList<Obstacle> obstacles, ArrayList<OctVirtualPointFixSlaveVar> octVirtualPointFixSlaveVars, ArrayList<PseudoBase> slaves, PseudoBase master, GurobiVariable busMin, GurobiVariable busXY, GurobiVariable branchMin, GurobiVariable branchXY, GurobiExecutor executor, int m, int minDist) {
        super(obstacles, slaves, master, executor, m, minDist);

        this.octVirtualPointFixSlaveVars = octVirtualPointFixSlaveVars;

        this.busMin = busMin;
        this.busXY = busXY;
        this.branchMin = branchMin;
        this.branchXY = branchXY;

    }

    public void buildConstraints() throws GRBException {
        GurobiConstraint c;
        OctVirtualPointFixSlaveVar vpN;

        //busLength Min
        GurobiConstraint c_busMin = new GurobiConstraint();
        c_busMin.setName("c_busMin");
        c_busMin.addToLHS(busMin, 1.0);
        c_busMin.setSense('=');
        executor.addConstraint(c_busMin);
        //busLength Difference
        GurobiConstraint c_busXY = new GurobiConstraint();
        c_busXY.setName("c_busXY");
        c_busXY.addToLHS(busXY, 1.0);
        c_busXY.setSense('=');
        executor.addConstraint(c_busXY);
        //branchLength Min
        GurobiConstraint c_branchMin = new GurobiConstraint();
        c_branchMin.setName("c_branchMin");
        c_branchMin.addToLHS(branchMin, 1.0);
        c_branchMin.setSense('=');
        executor.addConstraint(c_branchMin);
        //branchLength Difference
        GurobiConstraint c_branchXY = new GurobiConstraint();
        c_branchXY.setName("c_branchXY");
        c_branchXY.addToLHS(branchXY, 1.0);
        c_branchXY.setSense('=');
        executor.addConstraint(c_branchXY);

        for (OctVirtualPointFixSlaveVar vp : octVirtualPointFixSlaveVars){
            //corresponding slave
            PseudoBase sv = vp.slave;
            /*
            non-overlapping and orientation determination
             */
            for (Obstacle o : obstacles){
                buildCons_nonOverlapAndOppositeRelation(minDist, vp, o);

            }
            /*
            Index 0 -- vps.size() - 1:
            Connection with next virtualPoint
             */
            if (octVirtualPointFixSlaveVars.indexOf(vp) < octVirtualPointFixSlaveVars.size() - 1) {
                vpN = octVirtualPointFixSlaveVars.get(octVirtualPointFixSlaveVars.indexOf(vp) + 1);
                String name = "v" + octVirtualPointFixSlaveVars.indexOf(vp) + "->v" + (octVirtualPointFixSlaveVars.indexOf(vp)+1);


                //add To bus length:
                c_busMin.addToRHS(vp.dist_cqs[0], 1.0);
                c_busXY.addToRHS(vp.dist_cqs[1], 1.0);
                /*
                d_i_i+1 without Detour
                VVVV
                 */
                c = new GurobiConstraint();
                c.setName(name + "_pathLength.min");
                c.addToLHS(vp.dist_cqs[0], 1.0);
                c.setSense('>');
                c.addToRHS(vp.aux_dist_iqs[2], 1.0);
                executor.addConstraint(c);
                c = new GurobiConstraint();
                c.setName(name + "_pathLength.xy");
                c.addToLHS(vp.dist_cqs[1], 1.0);
                c.setSense('>');
                c.addToRHS(vp.aux_dist_iqs[3], 1.0);
                executor.addConstraint(c);

                auxiliaryDistCons(name + "_d_woDetour", vp.x, vpN.x, vp.y, vpN.y,  vp.aux_dist_iqs, vp.auxQ_dist);
                /*
                AAAA
                d_i_i+1
                 */

                //d_ij regarding Detour
                GurobiConstraint c_pathLengthMin = new GurobiConstraint();
                c_pathLengthMin.setName(name + "_pathLength_detour_min");
                c_pathLengthMin.addToLHS(vp.dist_cqs[0], 1.0);
                c_pathLengthMin.setSense('>');
                c_pathLengthMin.addToRHS(vp.dOut_cqs[0], 1.0);
                c_pathLengthMin.addToRHS(vp.dIn_cqs[0], 1.0);
                c_pathLengthMin.addToRHS(vp.detour_q, M);
                c_pathLengthMin.setRHSConstant(-M);
                executor.addConstraint(c_pathLengthMin);
                GurobiConstraint c_pathLengthXY = new GurobiConstraint();
                c_pathLengthXY.setName(name + "_pathLength_detour_xy");
                c_pathLengthXY.addToLHS(vp.dist_cqs[1], 1.0);
                c_pathLengthXY.setSense('>');
                c_pathLengthXY.addToRHS(vp.dOut_cqs[1], 1.0);
                c_pathLengthXY.addToRHS(vp.dIn_cqs[1], 1.0);
                c_pathLengthXY.addToRHS(vp.detour_q, M);
                c_pathLengthXY.setRHSConstant(-M);
                executor.addConstraint(c_pathLengthXY);

                //detour_triggering_aux.2: if one obstacle is relevant => detour trigger
                GurobiConstraint c_detour_geq = new GurobiConstraint();
                c_detour_geq.setName(name + "_detourTriggerAux2_geq");
                c_detour_geq.addToLHS(vp.detour_q, obstacles.size());
                c_detour_geq.setSense('>');
                executor.addConstraint(c_detour_geq);
                GurobiConstraint c_detour_leq = new GurobiConstraint();
                c_detour_leq.setName(name + "_detourTriggerAux2_leq");
                c_detour_leq.addToLHS(vp.detour_q, 1.0);
                c_detour_leq.setSense('<');
                executor.addConstraint(c_detour_leq);
                //cnnRules.5: if detour => startPoint should connect one of the relevant obstacle
                GurobiConstraint c_svpToObstacle = new GurobiConstraint();
                c_svpToObstacle.setName(name + "_cnnRules.5");
                c_svpToObstacle.setSense('=');
                c_svpToObstacle.addToRHS(vp.detour_q, 1.0);
                executor.addConstraint(c_svpToObstacle);
                //cnnRules.6: if detour => one of the relevant obstacle should connect to endPoint
                GurobiConstraint c_obstacleToEvp = new GurobiConstraint();
                c_obstacleToEvp.setName(name + "_cnnRules.6");
                c_obstacleToEvp.setSense('=');
                c_obstacleToEvp.addToRHS(vp.detour_q, 1.0);
                executor.addConstraint(c_obstacleToEvp);
                for (Obstacle o : obstacles){
                    String oName = name + "_" + o.getName();
                    c_detour_geq.addToRHS(vp.relObstacles_q.get(o), 1.0);
                    c_detour_leq.addToRHS(vp.relObstacles_q.get(o), 1.0);

                    //cnnRules.5_lin
                    linearizeProduct2BVars(vp.linAux_inOut_qs.get(o)[0], vp.startEndObstacles_qs.get(o)[0], vp.relObstacles_q.get(o));
                    c_svpToObstacle.addToLHS(vp.linAux_inOut_qs.get(o)[0], 1.0);



                    //cnnRules.6_lin
                    linearizeProduct2BVars(vp.linAux_inOut_qs.get(o)[1], vp.startEndObstacles_qs.get(o)[1], vp.relObstacles_q.get(o));
                    c_obstacleToEvp.addToLHS(vp.linAux_inOut_qs.get(o)[1], 1.0);

                    /*
                    Opposite Relations Detection between vi and vi+1
                     */
                    buildCons_oppositeRelations(oName, vp, vpN, o);

                    /*
                    CornerRules
                     */
                    c = new GurobiConstraint();
                    c.setName(oName + "_cornerRules1.enter");
                    c.addToLHS(vp.enterCorner_qs.get(o)[0], 1.0);
                    c.addToLHS(vp.enterCorner_qs.get(o)[1], 1.0);
                    c.addToLHS(vp.enterCorner_qs.get(o)[2], 1.0);
                    c.addToLHS(vp.enterCorner_qs.get(o)[3], 1.0);
                    c.setSense('=');
                    c.addToRHS(vp.relObstacles_q.get(o), 1.0);
                    executor.addConstraint(c);
                    c = new GurobiConstraint();
                    c.setName(oName + "_cornerRules1.leave");
                    c.addToLHS(vp.leaveCorner_qs.get(o)[0], 1.0);
                    c.addToLHS(vp.leaveCorner_qs.get(o)[1], 1.0);
                    c.addToLHS(vp.leaveCorner_qs.get(o)[2], 1.0);
                    c.addToLHS(vp.leaveCorner_qs.get(o)[3], 1.0);
                    c.setSense('=');
                    c.addToRHS(vp.relObstacles_q.get(o), 1.0);
                    executor.addConstraint(c);
                    //vi and vi+1 are in direct opposite relations, enter and leave corners should not be the same corners
                    for (int i = 0; i < 4; ++i){
                        c = new GurobiConstraint();
                        c.setName(oName + "_cornerRules2." + i);
                        c.addToLHS(vp.enterCorner_qs.get(o)[i], 1.0);
                        c.addToLHS(vp.leaveCorner_qs.get(o)[i], 1.0);
                        c.setSense('<');
                        c.addToRHS(vp.relObstaclesD_qs.get(o)[0], 1.0);
                        c.addToRHS(vp.relObstaclesD_qs.get(o)[1], 1.0);
                        c.addToRHS(vp.relObstaclesD_qs.get(o)[2], 1.0);
                        c.addToRHS(vp.relObstaclesD_qs.get(o)[3], 1.0);
                        executor.addConstraint(c);
                    }
                    //cornerSelection for vi
                    buildCons_cornerSelection(oName + "enter", vp.startEndObstacles_qs.get(o)[0], vp.enterCorner_qs.get(o), vp.relD_qs.get(o), vp.rel_qs.get(o));

                    //cornerSelection for vi+1
                    buildCons_cornerSelection(oName + "leave", vp.startEndObstacles_qs.get(o)[1], vp.leaveCorner_qs.get(o), vpN.relD_qs.get(o), vpN.rel_qs.get(o));

                    //cnnRules.1: q_om->om = 0
                    c = new GurobiConstraint();
                    c.setName(oName + "_cnnRules.1");
                    c.addToLHS(vp.omOnCnn_q.get(o).get(o), 1.0);
                    c.setSense('=');
                    c.setRHSConstant(0.0);
                    executor.addConstraint(c);

                    //cnnRules.3: om relevant => connect to one relevant on or endpoint
                    GurobiConstraint c_leaveObstacleCnn = new GurobiConstraint();
                    c_leaveObstacleCnn.setName(oName + "_cnnRules.3");
                    c_leaveObstacleCnn.addToLHS(vp.relObstacles_q.get(o), 1.0);
                    c_leaveObstacleCnn.setSense('=');
                    c_leaveObstacleCnn.addToRHS(vp.startEndObstacles_qs.get(o)[1], 1.0);
                    executor.addConstraint(c_leaveObstacleCnn);
                    //cnnRules.4: om relevant => be connected by one relevant on or startpoint
                    GurobiConstraint c_enterObstacleCnn = new GurobiConstraint();
                    c_enterObstacleCnn.setName(oName + "_cnnRules.4");
                    c_enterObstacleCnn.addToLHS(vp.relObstacles_q.get(o), 1.0);
                    c_enterObstacleCnn.setSense('=');
                    c_enterObstacleCnn.addToRHS(vp.startEndObstacles_qs.get(o)[0], 1.0);
                    executor.addConstraint(c_enterObstacleCnn);
                    //vp->vp: Om ----> On
                    for (Obstacle on : obstacles) {
                        if (!o.getName().equals(on.getName())) {
                            String onName = oName + "->" + on.getName();

                            //cnnRules.2: q_om->on + q_on->om <= 1
                            c = new GurobiConstraint();
                            c.setName(onName + "_cnnRules.2");
                            c.addToLHS(vp.omOnCnn_q.get(o).get(on), 1.0);
                            c.addToLHS(vp.omOnCnn_q.get(on).get(o), 1.0);
                            c.setSense('<');
                            c.setRHSConstant(1.0);
                            executor.addConstraint(c);

                            //cnnRules.3_lin
                            linearizeProduct2BVars(vp.linAux_OmOn_qs.get(o).get(on)[0], vp.omOnCnn_q.get(o).get(on), vp.relObstacles_q.get(on));
                            c_leaveObstacleCnn.addToRHS(vp.linAux_OmOn_qs.get(o).get(on)[0], 1.0);


                            //cnnRules.4_lin
                            linearizeProduct2BVars(vp.linAux_OmOn_qs.get(on).get(o)[1], vp.omOnCnn_q.get(on).get(o), vp.relObstacles_q.get(o));
                            c_enterObstacleCnn.addToRHS(vp.linAux_OmOn_qs.get(on).get(o)[1], 1.0);

                            //pathLength.m->n
                            c = new GurobiConstraint();
                            c.setName(onName + "_pathLength.m->n.min");
                            c.addToLHS(vp.dOmOn_cqs.get(o).get(on)[0], 1.0);
                            c.setSense('>');
                            c.addToRHS(vp.aux_dOmOn_iqs.get(o).get(on)[2], 1.0);
                            c.addToRHS(vp.omOnCnn_q.get(o).get(on), M);
                            c.setRHSConstant(-M);
                            executor.addConstraint(c);
                            c = new GurobiConstraint();
                            c.setName(onName + "_pathLength.m->n.xy");
                            c.addToLHS(vp.dOmOn_cqs.get(o).get(on)[1], 1.0);
                            c.setSense('>');
                            c.addToRHS(vp.aux_dOmOn_iqs.get(o).get(on)[3], 1.0);
                            c.addToRHS(vp.aux_dOm_iqs.get(o)[0], 1.0);
                            c.addToRHS(vp.aux_dOm_iqs.get(o)[1], 1.0);
                            c.addToRHS(vp.omOnCnn_q.get(o).get(on), M);
                            c.setRHSConstant(-M);
                            executor.addConstraint(c);
                            auxiliaryDistCons(onName + "_pathLength.m->n", vp.leaveCoordinate_iqs.get(o), vp.enterCoordinate_iqs.get(on), vp.aux_dOmOn_iqs.get(o).get(on), vp.auxQ_dOmOn.get(o).get(on));

                            c_pathLengthMin.addToRHS(vp.dOmOn_cqs.get(o).get(on)[0], 1.0);
                            c_pathLengthXY.addToRHS(vp.dOmOn_cqs.get(o).get(on)[1], 1.0);

                        }
                        auxiliaryManhattanDistCons(oName + "_enter->leave", vp.enterCoordinate_iqs.get(o), vp.leaveCoordinate_iqs.get(o), vp.aux_dOm_iqs.get(o));

                    }

                    //coordinate
                    buildCons_oCoordinate(oName + "_enter", o, vp.enterCoordinate_iqs.get(o), vp.enterCorner_qs.get(o), vp.relObstacles_q.get(o));
                    buildCons_oCoordinate(oName + "_leave", o, vp.leaveCoordinate_iqs.get(o), vp.leaveCorner_qs.get(o), vp.relObstacles_q.get(o));

                    //pathLength.stp->
                    c = new GurobiConstraint();
                    c.setName(oName + "_pathLength.stp->.min");
                    c.addToLHS(vp.dOut_cqs[0], 1.0);
                    c.setSense('>');
                    c.addToRHS(vp.aux_dOut_iqs.get(o)[2], 1.0);
                    c.addToRHS(vp.startEndObstacles_qs.get(o)[0], M);
                    c.setRHSConstant(-M);
                    executor.addConstraint(c);
                    c = new GurobiConstraint();
                    c.setName(oName + "_pathLength.stp->.xy");
                    c.addToLHS(vp.dOut_cqs[1], 1.0);
                    c.setSense('>');
                    c.addToRHS(vp.aux_dOut_iqs.get(o)[3], 1.0);
                    c.addToRHS(vp.startEndObstacles_qs.get(o)[0], M);
                    c.setRHSConstant(-M);
                    executor.addConstraint(c);
                    auxiliaryDistCons(oName + "_pathLength.stp->", vp.x, vp.enterCoordinate_iqs.get(o)[0], vp.y, vp.enterCoordinate_iqs.get(o)[1], vp.aux_dOut_iqs.get(o), vp.auxQ_dOut.get(o));

                    //pathLength.->Evp
                    c = new GurobiConstraint();
                    c.setName(oName + "_pathLength.->Evp.min");
                    c.addToLHS(vp.dIn_cqs[0], 1.0);
                    c.setSense('>');
                    c.addToRHS(vp.aux_dIn_iqs.get(o)[2], 1.0);
                    c.addToRHS(vp.startEndObstacles_qs.get(o)[1], M);
                    c.setRHSConstant(-M);
                    executor.addConstraint(c);
                    c = new GurobiConstraint();
                    c.setName(oName + "_pathLength.->Evp.xy");
                    c.addToLHS(vp.dIn_cqs[1], 1.0);
                    c.setSense('>');
                    c.addToRHS(vp.aux_dIn_iqs.get(o)[3], 1.0);
                    c.addToRHS(vp.startEndObstacles_qs.get(o)[1], M);
                    c.setRHSConstant(-M);
                    executor.addConstraint(c);
                    auxiliaryDistCons(oName + "_pathLength.->Evp", vp.leaveCoordinate_iqs.get(o)[0], vpN.x, vp.leaveCoordinate_iqs.get(o)[1], vpN.y, vp.aux_dIn_iqs.get(o), vp.auxQ_dIn.get(o));
                }
            }

            /*
            vp -> slave
             */
            String sName = "vs" + octVirtualPointFixSlaveVars.indexOf(vp);
            //add to branchLength
            c_branchMin.addToRHS(vp.vs_dist_cqs[0], 1.0);
            c_branchXY.addToRHS(vp.vs_dist_cqs[1], 1.0);

            buildCons_vpToBase(sName, vp, sv, vp.vs_dist_cqs, vp.aux_vs_dist_iqs, vp.auxQ_vs_dist, vp.vs_dOut_cqs, vp.vs_dIn_cqs, vp.vs_detour_q, vp.vs_relObstacles_q, vp.linAux_vsInOut_qs, vp.vs_startEndObstacles_qs, vp.vs_relObstacles_qs, vp.vs_relObstaclesD_qs, vp.vs_enterCorner_qs, vp.vs_leaveCorner_qs, vp.vs_omOnCnn_q, vp.linAux_vsOmOn_qs, vp.vs_dOmOn_cqs, vp.aux_vsdOmOn_iqs, vp.auxQ_vsdOmOn, vp.aux_vsdOm_iqs, vp.vs_enterCoordinate_iqs, vp.vs_leaveCoordinate_iqs, vp.aux_vsdOut_iqs, vp.auxQ_vsdOut, vp.aux_vsdIn_iqs, vp.auxQ_vsdIn);

            /*
            v0 -> master
             */
            if (vp instanceof OctFirstVirtualPointFixSlaveVar){
                String mName = "vm";
                OctFirstVirtualPointFixSlaveVar vF = (OctFirstVirtualPointFixSlaveVar) vp;
                //add to busLength
                c_busMin.addToRHS(vF.vm_dist_cqs[0], 1.0);
                c_busXY.addToRHS(vF.vm_dist_cqs[1], 1.0);

                buildCons_vpToBase(mName, vF, master, vF.vm_dist_cqs, vF.aux_vm_dist_iqs, vF.auxQ_vm_dist, vF.vm_dOut_cqs, vF.vm_dIn_cqs, vF.vm_detour_q, vF.vm_relObstacles_q, vF.linAux_vmInOut_qs, vF.vm_startEndObstacles_qs, vF.vm_relObstacles_qs, vF.vm_relObstaclesD_qs, vF.vm_enterCorner_qs, vF.vm_leaveCorner_qs, vF.vm_omOnCnn_q, vF.linAux_vmOmOn_qs, vF.vm_dOmOn_cqs, vF.aux_vmdOmOn_iqs, vF.auxQ_vmdOmOn, vF.aux_vmdOm_iqs, vF.vm_enterCoordinate_iqs, vF.vm_leaveCoordinate_iqs, vF.aux_vmdOut_iqs, vF.auxQ_vmdOut, vF.aux_vmdIn_iqs, vF.auxQ_vmdIn);
            }

        }





    }

    private void buildCons_vpToBase(String nickname, OctVirtualPointFixSlaveVar vp, PseudoBase base, GurobiVariable[] dist_cqs, GurobiVariable[] aux_dist_iqs, GurobiVariable auxQ_dist, GurobiVariable[] dOut_cqs, GurobiVariable[] dIn_cqs, GurobiVariable detour_q, Map<Obstacle, GurobiVariable> relObstacles_q, Map<Obstacle, GurobiVariable[]> linAux_inOut_qs, Map<Obstacle, GurobiVariable[]> startEndObstacles_qs, Map<Obstacle, GurobiVariable[]> relObstacles_qs, Map<Obstacle, GurobiVariable[]> relObstaclesD_qs, Map<Obstacle, GurobiVariable[]> enterCorner_qs, Map<Obstacle, GurobiVariable[]> leaveCorner_qs, Map<Obstacle, Map<Obstacle, GurobiVariable>> omOnCnn_q, Map<Obstacle, Map<Obstacle, GurobiVariable[]>> linAux_OmOn_qs, Map<Obstacle, Map<Obstacle, GurobiVariable[]>> dOmOn_cqs, Map<Obstacle, Map<Obstacle, GurobiVariable[]>> aux_dOmOn_iqs, Map<Obstacle, Map<Obstacle, GurobiVariable>> auxQ_dOmOn, Map<Obstacle, GurobiVariable[]> aux_dOm_iqs, Map<Obstacle, GurobiVariable[]> enterCoordinate_iqs, Map<Obstacle, GurobiVariable[]> leaveCoordinate_iqs, Map<Obstacle, GurobiVariable[]> aux_dOut_iqs, Map<Obstacle, GurobiVariable> auxQ_dOut, Map<Obstacle, GurobiVariable[]> aux_dIn_iqs, Map<Obstacle, GurobiVariable> auxQ_dIn) throws GRBException {
        GurobiConstraint c;
        /*
        d_i_Base without detour
        VVVV
         */
        c = new GurobiConstraint();
        c.setName(nickname + "_pathLength.min");
        c.addToLHS(dist_cqs[0], 1.0);
        c.setSense('>');
        c.addToRHS(aux_dist_iqs[2], 1.0);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName(nickname + "_pathLength.xy");
        c.addToLHS(dist_cqs[1], 1.0);
        c.setSense('>');
        c.addToRHS(aux_dist_iqs[3], 1.0);
        executor.addConstraint(c);

        auxiliaryDistCons(nickname + "_d_woDetour", vp.x, vp.y, base, aux_dist_iqs, auxQ_dist);
        /*
        AAAA
        d_i_Base
         */

        //d_i_Base regarding detour
        GurobiConstraint c_base_pathLengthMin = new GurobiConstraint();
        c_base_pathLengthMin.setName(nickname + "_pathLength_detour_min");
        c_base_pathLengthMin.addToLHS(dist_cqs[0], 1.0);
        c_base_pathLengthMin.setSense('>');
        c_base_pathLengthMin.addToRHS(dOut_cqs[0], 1.0);
        c_base_pathLengthMin.addToRHS(dIn_cqs[0], 1.0);
        c_base_pathLengthMin.addToRHS(detour_q, M);
        c_base_pathLengthMin.setRHSConstant(-M);
        executor.addConstraint(c_base_pathLengthMin);
        GurobiConstraint c_base_pathLengthXY = new GurobiConstraint();
        c_base_pathLengthXY.setName(nickname + "_pathLength_detour_xy");
        c_base_pathLengthXY.addToLHS(dist_cqs[1], 1.0);
        c_base_pathLengthXY.setSense('>');
        c_base_pathLengthXY.addToRHS(dOut_cqs[1], 1.0);
        c_base_pathLengthXY.addToRHS(dIn_cqs[1], 1.0);
        c_base_pathLengthXY.addToRHS(detour_q, M);
        c_base_pathLengthXY.setRHSConstant(-M);
        executor.addConstraint(c_base_pathLengthXY);

        //vp->Base: detour_triggering_aux.2: if one obstacle is relevant => detour trigger
        GurobiConstraint c_base_detour_geq = new GurobiConstraint();
        c_base_detour_geq.setName(nickname + "_detourTriggerAux2_geq");
        c_base_detour_geq.addToLHS(detour_q, obstacles.size());
        c_base_detour_geq.setSense('>');
        executor.addConstraint(c_base_detour_geq);
        GurobiConstraint c_base_detour_leq = new GurobiConstraint();
        c_base_detour_leq.setName(nickname + "_detourTriggerAux2_leq");
        c_base_detour_leq.addToLHS(detour_q, 1.0);
        c_base_detour_leq.setSense('<');
        executor.addConstraint(c_base_detour_leq);
        //vp->Base: cnnRules.5: if detour => startPoint should connect one of the relevant obstacle
        GurobiConstraint c_base_svpToObstacle = new GurobiConstraint();
        c_base_svpToObstacle.setName(nickname + "_cnnRules.5");
        c_base_svpToObstacle.setSense('=');
        c_base_svpToObstacle.addToRHS(detour_q, 1.0);
        executor.addConstraint(c_base_svpToObstacle);
        //vp->Base: cnnRules.6: if detour => one of the relevant obstacle should connect to endPoint
        GurobiConstraint c_base_obstacleToEvp = new GurobiConstraint();
        c_base_obstacleToEvp.setName(nickname + "_cnnRules.6");
        c_base_obstacleToEvp.setSense('=');
        c_base_obstacleToEvp.addToRHS(detour_q, 1.0);
        executor.addConstraint(c_base_obstacleToEvp);

        for (Obstacle o : obstacles){
            String oNickname = nickname + "_" + o.getName();
            c_base_detour_geq.addToRHS(relObstacles_q.get(o), 1.0);
            c_base_detour_leq.addToRHS(relObstacles_q.get(o), 1.0);

            //vp->Base: cnnRules.5_lin
            linearizeProduct2BVars(linAux_inOut_qs.get(o)[0], startEndObstacles_qs.get(o)[0], relObstacles_q.get(o));
            c_base_svpToObstacle.addToLHS(linAux_inOut_qs.get(o)[0], 1.0);

            //vp->Base: cnnRules.6_lin
            linearizeProduct2BVars(linAux_inOut_qs.get(o)[1], startEndObstacles_qs.get(o)[1], relObstacles_q.get(o));
            c_base_obstacleToEvp.addToLHS(linAux_inOut_qs.get(o)[1], 1.0);

            /*
            vp->Base: opposite relations detection between vi and si
             */
            buildCons_oppositeRelations(oNickname, vp, base, o, relObstacles_qs, relObstaclesD_qs);

            /*
            vp->Base: CornerRules
             */
            c = new GurobiConstraint();
            c.setName(oNickname + "_cornerRules1.enter");
            c.addToLHS(enterCorner_qs.get(o)[0], 1.0);
            c.addToLHS(enterCorner_qs.get(o)[1], 1.0);
            c.addToLHS(enterCorner_qs.get(o)[2], 1.0);
            c.addToLHS(enterCorner_qs.get(o)[3], 1.0);
            c.setSense('=');
            c.addToRHS(relObstacles_q.get(o), 1.0);
            executor.addConstraint(c);
            c = new GurobiConstraint();
            c.setName(oNickname + "_cornerRules1.leave");
            c.addToLHS(leaveCorner_qs.get(o)[0], 1.0);
            c.addToLHS(leaveCorner_qs.get(o)[1], 1.0);
            c.addToLHS(leaveCorner_qs.get(o)[2], 1.0);
            c.addToLHS(leaveCorner_qs.get(o)[3], 1.0);
            c.setSense('=');
            c.addToRHS(relObstacles_q.get(o), 1.0);
            executor.addConstraint(c);
            //vp->Base: vi and base are in direct opposite relations, enter and leave corners should not be the same corners
            for (int i = 0; i < 4; ++i){
                c = new GurobiConstraint();
                c.setName(oNickname + "_cornerRules2." + i);
                c.addToLHS(enterCorner_qs.get(o)[i], 1.0);
                c.addToLHS(leaveCorner_qs.get(o)[i], 1.0);
                c.setSense('<');
                c.addToRHS(relObstaclesD_qs.get(o)[0], 1.0);
                c.addToRHS(relObstaclesD_qs.get(o)[1], 1.0);
                c.addToRHS(relObstaclesD_qs.get(o)[2], 1.0);
                c.addToRHS(relObstaclesD_qs.get(o)[3], 1.0);
                executor.addConstraint(c);
            }
            //vp->Base: cornerSelection for vi -> Obstacle
            buildCons_cornerSelection(oNickname + "_enter", startEndObstacles_qs.get(o)[0], enterCorner_qs.get(o), vp.relD_qs.get(o), vp.rel_qs.get(o));
            //vp->Base: cornerSelection for Obstacle -> base
            int[] rel_q = Arrays.copyOfRange(base.getPseudo_oRel_qs().get(o), 0, 4);
            int[] relD_q = Arrays.copyOfRange(base.getPseudo_oRel_qs().get(o), base.getPseudo_oRel_qs().get(o).length - 4, base.getPseudo_oRel_qs().get(o).length);
            buildCons_cornerSelection(oNickname + "_leave", startEndObstacles_qs.get(o)[1], leaveCorner_qs.get(o), relD_q, rel_q);

            //vp->Base: cnnRules.1: q_om->om = 0
            c = new GurobiConstraint();
            c.setName(oNickname + "_cnnRules.1");
            c.addToLHS(omOnCnn_q.get(o).get(o), 1.0);
            c.setSense('=');
            c.setRHSConstant(0.0);
            executor.addConstraint(c);

            //vp->Base: cnnRules.3: om relevant => connect to one relevant on or endpoint
            GurobiConstraint c_base_leaveObstacleCnn = new GurobiConstraint();
            c_base_leaveObstacleCnn.setName(oNickname + "_cnnRules.3");
            c_base_leaveObstacleCnn.addToLHS(relObstacles_q.get(o), 1.0);
            c_base_leaveObstacleCnn.setSense('=');
            c_base_leaveObstacleCnn.addToRHS(startEndObstacles_qs.get(o)[1], 1.0);
            executor.addConstraint(c_base_leaveObstacleCnn);
            //vp->Base: cnnRules.4: om relevant => be connected by one relevant on or startpoint
            GurobiConstraint c_base_enterObstacleCnn = new GurobiConstraint();
            c_base_enterObstacleCnn.setName(oNickname + "_cnnRules.4");
            c_base_enterObstacleCnn.addToLHS(relObstacles_q.get(o), 1.0);
            c_base_enterObstacleCnn.setSense('=');
            c_base_enterObstacleCnn.addToRHS(startEndObstacles_qs.get(o)[0], 1.0);
            executor.addConstraint(c_base_enterObstacleCnn);
            //vp->Base: Om ----> On
            for (Obstacle on : obstacles){
                if (!o.getName().equals(on.getName())){
                    String sonName = oNickname + "->" + on.getName();

                    //vp->Base: cnnRules.2: q_om->on + q_on->om <= 1
                    c = new GurobiConstraint();
                    c.setName(sonName + "_cnnRules.2");
                    c.addToLHS(omOnCnn_q.get(o).get(on), 1.0);
                    c.addToLHS(omOnCnn_q.get(on).get(o), 1.0);
                    c.setSense('<');
                    c.setRHSConstant(1.0);
                    executor.addConstraint(c);

                    //vp->Base: cnnRules.3_lin
                    linearizeProduct2BVars(linAux_OmOn_qs.get(o).get(on)[0], omOnCnn_q.get(o).get(on), relObstacles_q.get(on));
                    c_base_leaveObstacleCnn.addToRHS(linAux_OmOn_qs.get(o).get(on)[0], 1.0);

                    //vp->Base: cnnRules.4_lin
                    linearizeProduct2BVars(linAux_OmOn_qs.get(on).get(o)[1], omOnCnn_q.get(on).get(o), relObstacles_q.get(o));
                    c_base_enterObstacleCnn.addToRHS(linAux_OmOn_qs.get(on).get(o)[1], 1.0);

                    //vp->Base: pathLength.m->n
                    c = new GurobiConstraint();
                    c.setName(sonName + "_pathLength.m->n.min");
                    c.addToLHS(dOmOn_cqs.get(o).get(on)[0], 1.0);
                    c.setSense('>');
                    c.addToRHS(aux_dOmOn_iqs.get(o).get(on)[2], 1.0);
                    c.addToRHS(omOnCnn_q.get(o).get(on), M);
                    c.setRHSConstant(-M);
                    executor.addConstraint(c);
                    c = new GurobiConstraint();
                    c.setName(sonName + "_pathLength.m->n.xy");
                    c.addToLHS(dOmOn_cqs.get(o).get(on)[1], 1.0);
                    c.setSense('>');
                    c.addToRHS(aux_dOmOn_iqs.get(o).get(on)[3], 1.0);
                    c.addToRHS(aux_dOm_iqs.get(o)[0], 1.0);
                    c.addToRHS(aux_dOm_iqs.get(o)[1], 1.0);
                    c.addToRHS(omOnCnn_q.get(o).get(on), M);
                    c.setRHSConstant(-M);
                    executor.addConstraint(c);
                    auxiliaryDistCons(sonName + "_pathLength.m->n", leaveCoordinate_iqs.get(o), enterCoordinate_iqs.get(on), aux_dOmOn_iqs.get(o).get(on), auxQ_dOmOn.get(o).get(on));

                    c_base_pathLengthMin.addToRHS(dOmOn_cqs.get(o).get(on)[0], 1.0);
                    c_base_pathLengthXY.addToRHS(dOmOn_cqs.get(o).get(on)[1], 1.0);


                }
                auxiliaryManhattanDistCons(oNickname + "_enter->leave", enterCoordinate_iqs.get(o), leaveCoordinate_iqs.get(o), aux_dOm_iqs.get(o));

            }

            //vp->Base: coordinate
            buildCons_oCoordinate(oNickname + "_enter", o, enterCoordinate_iqs.get(o), enterCorner_qs.get(o), relObstacles_q.get(o));
            buildCons_oCoordinate(oNickname + "_leave", o, leaveCoordinate_iqs.get(o), leaveCorner_qs.get(o), relObstacles_q.get(o));

            //vp->Base: pathLength.stp->
            c = new GurobiConstraint();
            c.setName(oNickname + "_pathLength.stp->.min");
            c.addToLHS(dOut_cqs[0], 1.0);
            c.setSense('>');
            c.addToRHS(aux_dOut_iqs.get(o)[2], 1.0);
            c.addToRHS(startEndObstacles_qs.get(o)[0], M);
            c.setRHSConstant(-M);
            executor.addConstraint(c);
            c = new GurobiConstraint();
            c.setName(oNickname + "_pathLength.stp->.xy");
            c.addToLHS(dOut_cqs[1], 1.0);
            c.setSense('>');
            c.addToRHS(aux_dOut_iqs.get(o)[3], 1.0);
            c.addToRHS(startEndObstacles_qs.get(o)[0], M);
            c.setRHSConstant(-M);
            executor.addConstraint(c);
            auxiliaryDistCons(oNickname + "_pathLength.stp->", vp.x, enterCoordinate_iqs.get(o)[0], vp.y, enterCoordinate_iqs.get(o)[1], aux_dOut_iqs.get(o), auxQ_dOut.get(o));

            //vp->Base: pathLength.->Evp
            c = new GurobiConstraint();
            c.setName(oNickname + "_pathLength.->Evp.min");
            c.addToLHS(dIn_cqs[0], 1.0);
            c.setSense('>');
            c.addToRHS(aux_dIn_iqs.get(o)[2], 1.0);
            c.addToRHS(startEndObstacles_qs.get(o)[1], M);
            c.setRHSConstant(-M);
            executor.addConstraint(c);
            c = new GurobiConstraint();
            c.setName(oNickname + "_pathLength.->Evp.xy");
            c.addToLHS(dIn_cqs[1], 1.0);
            c.setSense('>');
            c.addToRHS(aux_dIn_iqs.get(o)[3], 1.0);
            c.addToRHS(startEndObstacles_qs.get(o)[1], M);
            c.setRHSConstant(-M);
            executor.addConstraint(c);
            auxiliaryDistCons(oNickname + "_pathLength.->Evp", leaveCoordinate_iqs.get(o)[0], leaveCoordinate_iqs.get(o)[1], base, aux_dIn_iqs.get(o), auxQ_dIn.get(o));

        }
    }

    /**
     * Coordinates of the enter and leave corners
     * @param nickName name
     * @param o obstacle
     * @param coordinates [x y]
     * @param corner_qs binary variables for corner selcetion
     * @param relObstacles_q if the obstacle is relevant
     */
    private void buildCons_oCoordinate(String nickName, Obstacle o, GurobiVariable[] coordinates, GurobiVariable[] corner_qs, GurobiVariable relObstacles_q) {
        GurobiConstraint c;
        //todo: we may try "="
        c = new GurobiConstraint();
        c.setName(nickName + "Cor.x_leq");
        c.addToLHS(coordinates[0], 1.0);
        c.setSense('<');
        c.addToRHS(corner_qs[0], o.getMinX());
        c.addToRHS(corner_qs[1], o.getMinX());
        c.addToRHS(corner_qs[2], o.getMaxX());
        c.addToRHS(corner_qs[3], o.getMaxX());
        c.addToRHS(relObstacles_q, -M);
        c.setRHSConstant(M);
        c = new GurobiConstraint();
        c.setName(nickName + "Cor.x_geq");
        c.addToLHS(coordinates[0], 1.0);
        c.setSense('>');
        c.addToRHS(corner_qs[0], o.getMinX());
        c.addToRHS(corner_qs[1], o.getMinX());
        c.addToRHS(corner_qs[2], o.getMaxX());
        c.addToRHS(corner_qs[3], o.getMaxX());
        c.addToRHS(relObstacles_q, M);
        c.setRHSConstant(-M);
        c = new GurobiConstraint();
        c.setName(nickName + "Cor.y_leq");
        c.addToLHS(coordinates[1], 1.0);
        c.setSense('<');
        c.addToRHS(corner_qs[0], o.getMinY());
        c.addToRHS(corner_qs[3], o.getMinY());
        c.addToRHS(corner_qs[1], o.getMaxY());
        c.addToRHS(corner_qs[2], o.getMaxY());
        c.addToRHS(relObstacles_q, -M);
        c.setRHSConstant(M);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName(nickName + "Cor.y_geq");
        c.addToLHS(coordinates[1], 1.0);
        c.setSense('>');
        c.addToRHS(corner_qs[0], o.getMinY());
        c.addToRHS(corner_qs[3], o.getMinY());
        c.addToRHS(corner_qs[1], o.getMaxY());
        c.addToRHS(corner_qs[2], o.getMaxY());
        c.addToRHS(relObstacles_q, M);
        c.setRHSConstant(-M);
        executor.addConstraint(c);
    }

    /**
     * cornerRules.3 vp->obstacle or obstacle->vp
     * @param cName name
     * @param startOrEndObstacles_qs Gurobi variable indicate vi-> or ->vi+1
     * @param corner_qs enter or leave corner
     * @param relD_qs
     * @param rel_qs
     */
    private void buildCons_cornerSelection(String cName, GurobiVariable startOrEndObstacles_qs,GurobiVariable[] corner_qs, GurobiVariable[] relD_qs, GurobiVariable[] rel_qs) {
        GurobiConstraint c;
        c = new GurobiConstraint();
        c.setName(cName + "_corner.L");
        c.addToLHS(corner_qs[2], 1.0);
        c.addToLHS(corner_qs[3], 1.0);
        c.setSense('<');
        c.addToRHS(relD_qs[0], -1.0);
        c.addToRHS(startOrEndObstacles_qs, -1.0);
        c.setRHSConstant(2.0);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName(cName + "_corner.R");
        c.addToLHS(corner_qs[0], 1.0);
        c.addToLHS(corner_qs[1], 1.0);
        c.setSense('<');
        c.addToRHS(relD_qs[1], -1.0);
        c.addToRHS(startOrEndObstacles_qs, -1.0);
        c.setRHSConstant(2.0);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName(cName + "_corner.T");
        c.addToLHS(corner_qs[0], 1.0);
        c.addToLHS(corner_qs[3], 1.0);
        c.setSense('<');
        c.addToRHS(relD_qs[2], -1.0);
        c.addToRHS(startOrEndObstacles_qs, -1.0);
        c.setRHSConstant(2.0);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName(cName + "_corner.B");
        c.addToLHS(corner_qs[1], 1.0);
        c.addToLHS(corner_qs[2], 1.0);
        c.setSense('<');
        c.addToRHS(relD_qs[3], -1.0);
        c.addToRHS(startOrEndObstacles_qs, -1.0);
        c.setRHSConstant(2.0);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName(cName + "_corner.tL");
        c.addToLHS(corner_qs[3], 1.0);
        c.setSense('<');
        c.addToRHS(rel_qs[0], -1.0);
        c.addToRHS(startOrEndObstacles_qs, -1.0);
        c.setRHSConstant(2.0);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName(cName + "_corner.tR");
        c.addToLHS(corner_qs[0], 1.0);
        c.setSense('<');
        c.addToRHS(rel_qs[1], -1.0);
        c.addToRHS(startOrEndObstacles_qs, -1.0);
        c.setRHSConstant(2.0);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName(cName + "_corner.bL");
        c.addToLHS(corner_qs[2], 1.0);
        c.setSense('<');
        c.addToRHS(rel_qs[2], -1.0);
        c.addToRHS(startOrEndObstacles_qs, -1.0);
        c.setRHSConstant(2.0);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName(cName + "_corner.bR");
        c.addToLHS(corner_qs[1], 1.0);
        c.setSense('<');
        c.addToRHS(rel_qs[3], -1.0);
        c.addToRHS(startOrEndObstacles_qs, -1.0);
        c.setRHSConstant(2.0);
        executor.addConstraint(c);
    }
    private void buildCons_cornerSelection(String cName, GurobiVariable startOrEndObstacles_qs,GurobiVariable[] corner_qs, int[] relD_qs, int[] rel_qs) {
        GurobiConstraint c;
        c = new GurobiConstraint();
        c.setName(cName + "_corner.L");
        c.addToLHS(corner_qs[2], 1.0);
        c.addToLHS(corner_qs[3], 1.0);
        c.setSense('<');
        c.addToRHS(startOrEndObstacles_qs, -1.0);
        c.setRHSConstant(2.0 - relD_qs[0]);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName(cName + "_corner.R");
        c.addToLHS(corner_qs[0], 1.0);
        c.addToLHS(corner_qs[1], 1.0);
        c.setSense('<');
        c.addToRHS(startOrEndObstacles_qs, -1.0);
        c.setRHSConstant(2.0 - relD_qs[1]);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName(cName + "_corner.T");
        c.addToLHS(corner_qs[0], 1.0);
        c.addToLHS(corner_qs[3], 1.0);
        c.setSense('<');
        c.addToRHS(startOrEndObstacles_qs, -1.0);
        c.setRHSConstant(2.0 - relD_qs[2]);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName(cName + "_corner.B");
        c.addToLHS(corner_qs[1], 1.0);
        c.addToLHS(corner_qs[2], 1.0);
        c.setSense('<');
        c.addToRHS(startOrEndObstacles_qs, -1.0);
        c.setRHSConstant(2.0 - relD_qs[3]);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName(cName + "_corner.tL");
        c.addToLHS(corner_qs[3], 1.0);
        c.setSense('<');
        c.addToRHS(startOrEndObstacles_qs, -1.0);
        c.setRHSConstant(2.0 - rel_qs[0]);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName(cName + "_corner.tR");
        c.addToLHS(corner_qs[0], 1.0);
        c.setSense('<');
        c.addToRHS(startOrEndObstacles_qs, -1.0);
        c.setRHSConstant(2.0 - rel_qs[1]);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName(cName + "_corner.bL");
        c.addToLHS(corner_qs[2], 1.0);
        c.setSense('<');
        c.addToRHS(startOrEndObstacles_qs, -1.0);
        c.setRHSConstant(2.0 - rel_qs[2]);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName(cName + "_corner.bR");
        c.addToLHS(corner_qs[1], 1.0);
        c.setSense('<');
        c.addToRHS(startOrEndObstacles_qs, -1.0);
        c.setRHSConstant(2.0 - rel_qs[3]);
        executor.addConstraint(c);
    }

    /**
     * Constraints for opposite relations between vi and vi+1
     * @param nickname name
     * @param vp vi
     * @param vpN vi+1
     * @param o obstacle
     */
    private void buildCons_oppositeRelations(String nickname, OctVirtualPointFixSlaveVar vp, OctVirtualPointFixSlaveVar vpN, Obstacle o) {
        GurobiConstraint c;
        //(1)ul->lr
        //ul->lr.2_o
        c = new GurobiConstraint();
        c.setName(nickname + "_ul->lr.2_o");
        c.addToLHS(vp.relObstacles_qs.get(o)[0], 1.0);
        c.setSense('<');
        c.addToRHS(vp.rel_qs.get(o)[0], 1.0);
        executor.addConstraint(c);
        for (Obstacle on : o.get_bRObstacles()) {
            //ul->lr.1_o
            c = new GurobiConstraint();
            c.setName(nickname + "_ul->lr.1_o");
            c.addToLHS(vp.relObstacles_qs.get(o)[0], 1.0);
            c.setSense('>');
            c.addToRHS(vpN.rel_qs.get(on)[3], 1.0);
            c.addToRHS(vp.rel_qs.get(o)[0], 1.0);
            c.setRHSConstant(-1.0);
            executor.addConstraint(c);


            if (!o.getName().equals(on.getName())) {
                //ul->lr.1_on
                c = new GurobiConstraint();
                c.setName(nickname + "_ul->lr.1_on");
                c.addToLHS(vp.relObstacles_qs.get(on)[0], 1.0);
                c.setSense('>');
                c.addToRHS(vpN.rel_qs.get(on)[3], 1.0);
                c.addToRHS(vp.rel_qs.get(o)[0], 1.0);
                c.setRHSConstant(-1.0);
                executor.addConstraint(c);
                //ul->lr.2_on
                c = new GurobiConstraint();
                c.setName(nickname + "_ul->lr.2_on");
                c.addToLHS(vp.relObstacles_qs.get(on)[0], 2.0);
                c.setSense('<');
                c.addToRHS(vpN.rel_qs.get(on)[3], 1.0);
                c.addToRHS(vp.rel_qs.get(o)[0], 1.0);
                executor.addConstraint(c);
            }
        }
        //(2)lr->ul
        c = new GurobiConstraint();
        c.setName(nickname + "_lr->ul.2_o");
        c.addToLHS(vp.relObstacles_qs.get(o)[1], 1.0);
        c.setSense('<');
        c.addToRHS(vp.rel_qs.get(o)[3], 1.0);
        executor.addConstraint(c);
        for (Obstacle on : o.get_tLObstacles()) {
            //lr->ul.1_o
            c = new GurobiConstraint();
            c.setName(nickname + "_lr->ul.1_o");
            c.addToLHS(vp.relObstacles_qs.get(o)[1], 1.0);
            c.setSense('>');
            c.addToRHS(vpN.rel_qs.get(on)[0], 1.0);
            c.addToRHS(vp.rel_qs.get(o)[3], 1.0);
            c.setRHSConstant(-1.0);
            executor.addConstraint(c);

            if (!o.getName().equals(on.getName())) {
                //lr->ul.1_on
                c = new GurobiConstraint();
                c.setName(nickname + "_lr->ul.1_on");
                c.addToLHS(vp.relObstacles_qs.get(on)[1], 1.0);
                c.setSense('>');
                c.addToRHS(vpN.rel_qs.get(on)[0], 1.0);
                c.addToRHS(vp.rel_qs.get(o)[3], 1.0);
                c.setRHSConstant(-1.0);
                executor.addConstraint(c);

                //lr->ul.2_on
                c = new GurobiConstraint();
                c.setName(nickname + "_lr->ul.2_on");
                c.addToLHS(vp.relObstacles_qs.get(on)[1], 2.0);
                c.setSense('<');
                c.addToRHS(vpN.rel_qs.get(on)[0], 1.0);
                c.addToRHS(vp.rel_qs.get(o)[3], 1.0);
                executor.addConstraint(c);

            }
        }
        //(3)ur->ll
        c = new GurobiConstraint();
        c.setName(nickname + "_ur->ll.2_o");
        c.addToLHS(vp.relObstacles_qs.get(o)[2], 1.0);
        c.setSense('<');
        c.addToRHS(vp.rel_qs.get(o)[1], 1.0);
        executor.addConstraint(c);
        for (Obstacle on : o.get_bLObstacles()) {
            //ur->ll.1_o
            c = new GurobiConstraint();
            c.setName(nickname + "_ur->ll.1_o");
            c.addToLHS(vp.relObstacles_qs.get(o)[2], 1.0);
            c.setSense('>');
            c.addToRHS(vpN.rel_qs.get(on)[2], 1.0);
            c.addToRHS(vp.rel_qs.get(o)[1], 1.0);
            c.setRHSConstant(-1.0);
            executor.addConstraint(c);


            if (!o.getName().equals(on.getName())) {
                //ur->ll.1_on
                c = new GurobiConstraint();
                c.setName(nickname + "_ur->ll.1_on");
                c.addToLHS(vp.relObstacles_qs.get(on)[2], 1.0);
                c.setSense('>');
                c.addToRHS(vpN.rel_qs.get(on)[2], 1.0);
                c.addToRHS(vp.rel_qs.get(o)[1], 1.0);
                c.setRHSConstant(-1.0);
                executor.addConstraint(c);

                //ur->lr.2_on
                c = new GurobiConstraint();
                c.setName(nickname + "_ur->ll.2_on");
                c.addToLHS(vp.relObstacles_qs.get(on)[2], 2.0);
                c.setSense('<');
                c.addToRHS(vpN.rel_qs.get(on)[2], 1.0);
                c.addToRHS(vp.rel_qs.get(o)[1], 1.0);
                executor.addConstraint(c);
            }
        }
        //(4)ll->ur
        c = new GurobiConstraint();
        c.setName(nickname + "_ll->ur.2_o");
        c.addToLHS(vp.relObstacles_qs.get(o)[3], 1.0);
        c.setSense('<');
        c.addToRHS(vp.rel_qs.get(o)[2], 1.0);
        executor.addConstraint(c);
        for (Obstacle on : o.get_tRObstacles()) {
            //ll->ur.1_o
            c = new GurobiConstraint();
            c.setName(nickname + "_ll->ur.1_o");
            c.addToLHS(vp.relObstacles_qs.get(o)[3], 1.0);
            c.setSense('>');
            c.addToRHS(vpN.rel_qs.get(on)[1], 1.0);
            c.addToRHS(vp.rel_qs.get(o)[2], 1.0);
            c.setRHSConstant(-1.0);
            executor.addConstraint(c);

            if (!o.getName().equals(on.getName())) {
                //ll->ur.1_on
                c = new GurobiConstraint();
                c.setName(nickname + "_ll->ur.1_on");
                c.addToLHS(vp.relObstacles_qs.get(on)[3], 1.0);
                c.setSense('>');
                c.addToRHS(vpN.rel_qs.get(on)[1], 1.0);
                c.addToRHS(vp.rel_qs.get(o)[2], 1.0);
                c.setRHSConstant(-1.0);
                executor.addConstraint(c);

                //ll->ur.2_on
                c = new GurobiConstraint();
                c.setName(nickname + "_ll->ur.2_on");
                c.addToLHS(vp.relObstacles_qs.get(on)[3], 2.0);
                c.setSense('<');
                c.addToRHS(vpN.rel_qs.get(on)[1], 1.0);
                c.addToRHS(vp.rel_qs.get(o)[2], 1.0);
                executor.addConstraint(c);
            }
        }
        //(1)l->r
        c = new GurobiConstraint();
        c.setName(nickname + "_l->r.2_o");
        c.addToLHS(vp.relObstaclesD_qs.get(o)[0], 1.0);
        c.setSense('<');
        c.addToRHS(vp.relD_qs.get(o)[0], 1.0);
        executor.addConstraint(c);
        for (Obstacle on : o.get_dRObstacles()) {
            //(1)l->r.1_o
            c = new GurobiConstraint();
            c.setName(nickname + "_l->r.1_o");
            c.addToLHS(vp.relObstaclesD_qs.get(o)[0], 1.0);
            c.setSense('>');
            c.addToRHS(vpN.relD_qs.get(on)[1], 1.0);
            c.addToRHS(vp.relD_qs.get(o)[0], 1.0);
            c.setRHSConstant(-1.0);
            executor.addConstraint(c);

            if (!o.getName().equals(on.getName())) {
                //(1)l->r.1_on
                c = new GurobiConstraint();
                c.setName(nickname + "_l->r.1_on");
                c.addToLHS(vp.relObstaclesD_qs.get(on)[0], 1.0);
                c.setSense('>');
                c.addToRHS(vpN.relD_qs.get(on)[1], 1.0);
                c.addToRHS(vp.relD_qs.get(o)[0], 1.0);
                c.setRHSConstant(-1.0);
                executor.addConstraint(c);

                //(1)l->r.2_on
                c = new GurobiConstraint();
                c.setName(nickname + "_l->r.2_on");
                c.addToLHS(vp.relObstaclesD_qs.get(on)[0], 2.0);
                c.setSense('<');
                c.addToRHS(vpN.relD_qs.get(on)[1], 1.0);
                c.addToRHS(vp.relD_qs.get(o)[0], 1.0);
                executor.addConstraint(c);

            }


        }
        //(2)r->l
        c = new GurobiConstraint();
        c.setName(nickname + "_r->l.2_o");
        c.addToLHS(vp.relObstaclesD_qs.get(o)[1], 1.0);
        c.setSense('<');
        c.addToRHS(vp.relD_qs.get(o)[1], 1.0);
        executor.addConstraint(c);
        for (Obstacle on : o.get_dLObstacles()) {
            //(2)r->l.1_o
            c = new GurobiConstraint();
            c.setName(nickname + "_r->l.1_o");
            c.addToLHS(vp.relObstaclesD_qs.get(o)[1], 1.0);
            c.setSense('>');
            c.addToRHS(vpN.relD_qs.get(on)[0], 1.0);
            c.addToRHS(vp.relD_qs.get(o)[1], 1.0);
            c.setRHSConstant(-1.0);
            executor.addConstraint(c);
            if (!o.getName().equals(on.getName())) {

                //(2)r->l.1_on
                c = new GurobiConstraint();
                c.setName(nickname + "_r->l.1_o");
                c.addToLHS(vp.relObstaclesD_qs.get(on)[1], 1.0);
                c.setSense('>');
                c.addToRHS(vpN.relD_qs.get(on)[0], 1.0);
                c.addToRHS(vp.relD_qs.get(o)[1], 1.0);
                c.setRHSConstant(-1.0);
                executor.addConstraint(c);

                //(2)r->l.2_on
                c = new GurobiConstraint();
                c.setName(nickname + "_r->l.2_on");
                c.addToLHS(vp.relObstaclesD_qs.get(on)[1], 2.0);
                c.setSense('<');
                c.addToRHS(vpN.relD_qs.get(on)[0], 1.0);
                c.addToRHS(vp.relD_qs.get(o)[1], 1.0);
                executor.addConstraint(c);

            }


        }
        //(3)t->b
        c = new GurobiConstraint();
        c.setName(nickname + "_t->b.2_o");
        c.addToLHS(vp.relObstaclesD_qs.get(o)[2], 1.0);
        c.setSense('<');
        c.addToRHS(vp.relD_qs.get(o)[2], 1.0);
        executor.addConstraint(c);
        for (Obstacle on : o.get_dBObstacles()) {
            //(3)t->b.1_o
            c = new GurobiConstraint();
            c.setName(nickname + "_t->b.1_o");
            c.addToLHS(vp.relObstaclesD_qs.get(o)[2], 1.0);
            c.setSense('>');
            c.addToRHS(vpN.relD_qs.get(on)[3], 1.0);
            c.addToRHS(vp.relD_qs.get(o)[2], 1.0);
            c.setRHSConstant(-1.0);
            executor.addConstraint(c);
            if (!o.getName().equals(on.getName())) {

                //(3)t->b.1_on
                c = new GurobiConstraint();
                c.setName(nickname + "_t->b.1_on");
                c.addToLHS(vp.relObstaclesD_qs.get(on)[2], 1.0);
                c.setSense('>');
                c.addToRHS(vpN.relD_qs.get(on)[3], 1.0);
                c.addToRHS(vp.relD_qs.get(o)[2], 1.0);
                c.setRHSConstant(-1.0);
                executor.addConstraint(c);

                //(3)t->b.2_on
                c = new GurobiConstraint();
                c.setName("t->b.2_on");
                c.addToLHS(vp.relObstaclesD_qs.get(on)[2], 2.0);
                c.setSense('<');
                c.addToRHS(vpN.relD_qs.get(on)[3], 1.0);
                c.addToRHS(vp.relD_qs.get(o)[2], 1.0);
                executor.addConstraint(c);

            }


        }
        //(4)b->t
        c = new GurobiConstraint();
        c.setName("b->t.2_o");
        c.addToLHS(vp.relObstaclesD_qs.get(o)[3], 1.0);
        c.setSense('<');
        c.addToRHS(vp.relD_qs.get(o)[3], 1.0);
        executor.addConstraint(c);
        for (Obstacle on : o.get_dTObstacles()) {
            //(4) b->t.1_o
            c = new GurobiConstraint();
            c.setName(nickname + "_b->t.1_o");
            c.addToLHS(vp.relObstaclesD_qs.get(o)[3], 1.0);
            c.setSense('>');
            c.addToRHS(vpN.relD_qs.get(on)[2], 1.0);
            c.addToRHS(vp.relD_qs.get(o)[3], 1.0);
            c.setRHSConstant(-1.0);
            executor.addConstraint(c);

            if (!o.getName().equals(on.getName())) {
                //(4) b->t.1_on
                c = new GurobiConstraint();
                c.setName(nickname + "_b->t.1_on");
                c.addToLHS(vp.relObstaclesD_qs.get(on)[3], 1.0);
                c.setSense('>');
                c.addToRHS(vpN.relD_qs.get(on)[2], 1.0);
                c.addToRHS(vp.relD_qs.get(o)[3], 1.0);
                c.setRHSConstant(-1.0);
                executor.addConstraint(c);

                //(4) b->t.2_on
                c = new GurobiConstraint();
                c.setName(nickname + "_b->t.2_on");
                c.addToLHS(vp.relObstaclesD_qs.get(on)[3], 2.0);
                c.setSense('<');
                c.addToRHS(vpN.relD_qs.get(on)[2], 1.0);
                c.addToRHS(vp.relD_qs.get(o)[3], 1.0);
                executor.addConstraint(c);

            }


        }


        //Opposite Relation Recognition: aux.1
        c = new GurobiConstraint();
        c.setName("aux.1");
        c.addToLHS(vp.relObstacles_qs.get(o)[0], 1.0);
        c.addToLHS(vp.relObstacles_qs.get(o)[1], 1.0);
        c.addToLHS(vp.relObstacles_qs.get(o)[2], 1.0);
        c.addToLHS(vp.relObstacles_qs.get(o)[3], 1.0);
        c.addToLHS(vp.relObstaclesD_qs.get(o)[0], 1.0);
        c.addToLHS(vp.relObstaclesD_qs.get(o)[1], 1.0);
        c.addToLHS(vp.relObstaclesD_qs.get(o)[2], 1.0);
        c.addToLHS(vp.relObstaclesD_qs.get(o)[3], 1.0);
        c.setSense('<');
        c.addToRHS(vp.relObstacles_q.get(o), 8.0);
        executor.addConstraint(c);
    }

    private void buildCons_oppositeRelations(String nickname, OctVirtualPointFixSlaveVar vp, PseudoBase base, Obstacle o, Map<Obstacle, GurobiVariable[]> vpRelObstacles_qs, Map<Obstacle, GurobiVariable[]> vpRelObstaclesD_qs) {
        GurobiConstraint c;
        //1.ul->lr
        c = new GurobiConstraint();
        c.setName(nickname + "_ul->lr.2_o");
        c.addToLHS(vpRelObstacles_qs.get(o)[0], 1.0);
        c.setSense('<');
        c.addToRHS(vp.rel_qs.get(o)[0], 1.0);
        executor.addConstraint(c);
        for (Obstacle on : o.get_bRObstacles()) {
            //1.ul->lr.1_o
            c = new GurobiConstraint();
            c.setName(nickname + "_ul->lr.1_o");
            c.addToLHS(vpRelObstacles_qs.get(o)[0], 1.0);
            c.setSense('>');
            c.addToRHS(vp.rel_qs.get(o)[0], 1.0);
            c.setRHSConstant(base.getPseudo_oRel_qs().get(on)[3] - 1.0);
            executor.addConstraint(c);

            if (!o.getName().equals(on.getName())) {
                //1.ul->lr.1_on
                c = new GurobiConstraint();
                c.setName(nickname + "_ul->lr.1_on");
                c.addToLHS(vpRelObstacles_qs.get(on)[0], 1.0);
                c.setSense('>');
                c.addToRHS(vp.rel_qs.get(o)[0], 1.0);
                c.setRHSConstant(base.getPseudo_oRel_qs().get(on)[3] - 1.0);
                executor.addConstraint(c);
                //1.ul->lr.2_on
                c = new GurobiConstraint();
                c.setName(nickname + "_ul->lr.2_on");
                c.addToLHS(vpRelObstacles_qs.get(on)[0], 2.0);
                c.setSense('<');
                c.addToRHS(vp.rel_qs.get(o)[0], 1.0);
                c.setRHSConstant(base.getPseudo_oRel_qs().get(on)[3]);
                executor.addConstraint(c);
            }

        }

        //2.lr->ul
        c = new GurobiConstraint();
        c.setName(nickname + "_lr->ul.2_o");
        c.addToLHS(vpRelObstacles_qs.get(o)[1], 1.0);
        c.setSense('<');
        c.addToRHS(vp.rel_qs.get(o)[3], 1.0);
        executor.addConstraint(c);
        for (Obstacle on : o.get_tLObstacles()) {
            //2.lr->ul.1
            c = new GurobiConstraint();
            c.setName(nickname + "_lr->ul.1_o");
            c.addToLHS(vpRelObstacles_qs.get(o)[1], 1.0);
            c.setSense('>');
            c.addToRHS(vp.rel_qs.get(o)[3], 1.0);
            c.setRHSConstant(base.getPseudo_oRel_qs().get(on)[0] - 1.0);
            executor.addConstraint(c);

            if (!o.getName().equals(on.getName())) {
                //2.lr->ul.1_on
                c = new GurobiConstraint();
                c.setName(nickname + "_lr->ul.1_on");
                c.addToLHS(vpRelObstacles_qs.get(on)[1], 1.0);
                c.setSense('>');
                c.addToRHS(vp.rel_qs.get(o)[3], 1.0);
                c.setRHSConstant(base.getPseudo_oRel_qs().get(on)[0] - 1.0);
                executor.addConstraint(c);
                //2.lr->ul.2_on
                c = new GurobiConstraint();
                c.setName(nickname + "_lr->ul.2_on");
                c.addToLHS(vpRelObstacles_qs.get(on)[1], 2.0);
                c.setSense('<');
                c.addToRHS(vp.rel_qs.get(o)[3], 1.0);
                c.setRHSConstant(base.getPseudo_oRel_qs().get(on)[0]);
                executor.addConstraint(c);
            }

        }


        //3:ur->ll
        c = new GurobiConstraint();
        c.setName(nickname + "_ur->ll.2_o");
        c.addToLHS(vpRelObstacles_qs.get(o)[2], 1.0);
        c.setSense('<');
        c.addToRHS(vp.rel_qs.get(o)[1], 1.0);
        executor.addConstraint(c);
        for (Obstacle on : o.get_bLObstacles()) {
            //3:ur->ll.1
            c = new GurobiConstraint();
            c.setName(nickname + "_ur->ll.1_o");
            c.addToLHS(vpRelObstacles_qs.get(o)[2], 1.0);
            c.setSense('>');
            c.addToRHS(vp.rel_qs.get(o)[1], 1.0);
            c.setRHSConstant(base.getPseudo_oRel_qs().get(on)[2] - 1.0);
            executor.addConstraint(c);

            if (!o.getName().equals(on.getName())) {
                //3:ur->ll.1_on
                c = new GurobiConstraint();
                c.setName(nickname + "_ur->ll.1_on");
                c.addToLHS(vpRelObstacles_qs.get(on)[2], 1.0);
                c.setSense('>');
                c.addToRHS(vp.rel_qs.get(o)[1], 1.0);
                c.setRHSConstant(base.getPseudo_oRel_qs().get(on)[2] - 1.0);
                executor.addConstraint(c);
                //3:ur->ll.2_on
                c = new GurobiConstraint();
                c.setName(nickname + "_ur->ll.2_on");
                c.addToLHS(vpRelObstacles_qs.get(on)[2], 2.0);
                c.setSense('<');
                c.addToRHS(vp.rel_qs.get(o)[1], 1.0);
                c.setRHSConstant(base.getPseudo_oRel_qs().get(on)[2]);
                executor.addConstraint(c);
            }


        }


        //4:ll->ur
        c = new GurobiConstraint();
        c.setName(nickname + "_ll->ur.2_o");
        c.addToLHS(vpRelObstacles_qs.get(o)[3], 1.0);
        c.setSense('<');
        c.addToRHS(vp.rel_qs.get(o)[2], 1.0);
        executor.addConstraint(c);
        for (Obstacle on : o.get_tRObstacles()) {
            //4:ll->ur.1
            c = new GurobiConstraint();
            c.setName(nickname + "_ll->ur.1_o");
            c.addToLHS(vpRelObstacles_qs.get(o)[3], 1.0);
            c.setSense('>');
            c.addToRHS(vp.rel_qs.get(o)[2], 1.0);
            c.setRHSConstant(base.getPseudo_oRel_qs().get(on)[1] - 1.0);
            executor.addConstraint(c);


            if (!o.getName().equals(on.getName())) {
                //4:ll->ur.1_on
                c = new GurobiConstraint();
                c.setName(nickname + "_ll->ur.1_on");
                c.addToLHS(vpRelObstacles_qs.get(on)[3], 1.0);
                c.setSense('>');
                c.addToRHS(vp.rel_qs.get(o)[2], 1.0);
                c.setRHSConstant(base.getPseudo_oRel_qs().get(on)[1] - 1.0);
                executor.addConstraint(c);
                //4:ll->ur.2_on
                c = new GurobiConstraint();
                c.setName(nickname + "_ll->ur.2_on");
                c.addToLHS(vpRelObstacles_qs.get(on)[3], 2.0);
                c.setSense('<');
                c.addToRHS(vp.rel_qs.get(o)[2], 1.0);
                c.setRHSConstant(base.getPseudo_oRel_qs().get(on)[1]);
                executor.addConstraint(c);
            }

        }

        //(1)l->r
        c = new GurobiConstraint();
        c.setName(nickname + "_l->r.2_o");
        c.addToLHS(vpRelObstaclesD_qs.get(o)[0], 1.0);
        c.setSense('<');
        c.addToRHS(vp.relD_qs.get(o)[0], 1.0);
        executor.addConstraint(c);
        for (Obstacle on : o.get_dRObstacles()) {
            //(1)l->r.1_o
            c = new GurobiConstraint();
            c.setName(nickname + "_l->r.1_o");
            c.addToLHS(vpRelObstaclesD_qs.get(o)[0], 1.0);
            c.setSense('>');
            c.addToRHS(vp.relD_qs.get(o)[0], 1.0);
            c.setRHSConstant(base.getPseudo_oRel_qs().get(on)[5] - 1.0);
            executor.addConstraint(c);

            if (!o.getName().equals(on.getName())) {
                //(1)l->r.1_on
                c = new GurobiConstraint();
                c.setName(nickname + "_l->r.1_on");
                c.addToLHS(vpRelObstaclesD_qs.get(on)[0], 1.0);
                c.setSense('>');
                c.addToRHS(vp.relD_qs.get(o)[0], 1.0);
                c.setRHSConstant(base.getPseudo_oRel_qs().get(on)[5] - 1.0);
                executor.addConstraint(c);

                //(1)l->r.2_on
                c = new GurobiConstraint();
                c.setName(nickname + "_l->r.2_on");
                c.addToLHS(vpRelObstaclesD_qs.get(on)[0], 2.0);
                c.setSense('<');
                c.addToRHS(vp.relD_qs.get(o)[0], 1.0);
                c.setRHSConstant(base.getPseudo_oRel_qs().get(on)[5]);
                executor.addConstraint(c);

            }


        }


        //(2)r->l
        c = new GurobiConstraint();
        c.setName(nickname + "_r->l.2_o");
        c.addToLHS(vpRelObstaclesD_qs.get(o)[1], 1.0);
        c.setSense('<');
        c.addToRHS(vp.relD_qs.get(o)[1], 1.0);
        executor.addConstraint(c);
        for (Obstacle on : o.get_dLObstacles()) {
            //(2)r->l.1_o
            c = new GurobiConstraint();
            c.setName(nickname + "_r->l.1_o");
            c.addToLHS(vpRelObstaclesD_qs.get(o)[1], 1.0);
            c.setSense('>');
            c.addToRHS(vp.relD_qs.get(o)[1], 1.0);
            c.setRHSConstant(base.getPseudo_oRel_qs().get(on)[4] - 1.0);
            executor.addConstraint(c);
            if (!o.getName().equals(on.getName())) {

                //(2)r->l.1_on
                c = new GurobiConstraint();
                c.setName(nickname + "_r->l.1_o");
                c.addToLHS(vpRelObstaclesD_qs.get(on)[1], 1.0);
                c.setSense('>');
                c.addToRHS(vp.relD_qs.get(o)[1], 1.0);
                c.setRHSConstant(base.getPseudo_oRel_qs().get(on)[4] - 1.0);
                executor.addConstraint(c);

                //(2)r->l.2_on
                c = new GurobiConstraint();
                c.setName(nickname + "_r->l.2_on");
                c.addToLHS(vpRelObstaclesD_qs.get(on)[1], 2.0);
                c.setSense('<');
                c.addToRHS(vp.relD_qs.get(o)[1], 1.0);
                c.setRHSConstant(base.getPseudo_oRel_qs().get(on)[4]);
                executor.addConstraint(c);

            }


        }

        //(3)t->b
        c = new GurobiConstraint();
        c.setName(nickname + "_t->b.2_o");
        c.addToLHS(vpRelObstaclesD_qs.get(o)[2], 1.0);
        c.setSense('<');
        c.addToRHS(vp.relD_qs.get(o)[2], 1.0);
        executor.addConstraint(c);
        for (Obstacle on : o.get_dBObstacles()) {
            //(3)t->b.1_o
            c = new GurobiConstraint();
            c.setName(nickname + "_t->b.1_o");
            c.addToLHS(vpRelObstaclesD_qs.get(o)[2], 1.0);
            c.setSense('>');
            c.addToRHS(vp.relD_qs.get(o)[2], 1.0);
            c.setRHSConstant(base.getPseudo_oRel_qs().get(on)[7] - 1.0);
            executor.addConstraint(c);
            if (!o.getName().equals(on.getName())) {

                //(3)t->b.1_on
                c = new GurobiConstraint();
                c.setName(nickname + "_t->b.1_on");
                c.addToLHS(vpRelObstaclesD_qs.get(on)[2], 1.0);
                c.setSense('>');
                c.addToRHS(vp.relD_qs.get(o)[2], 1.0);
                c.setRHSConstant(base.getPseudo_oRel_qs().get(on)[7] - 1.0);
                executor.addConstraint(c);

                //(3)t->b.2_on
                c = new GurobiConstraint();
                c.setName(nickname + "_t->b.2_on");
                c.addToLHS(vpRelObstaclesD_qs.get(on)[2], 2.0);
                c.setSense('<');
                c.addToRHS(vp.relD_qs.get(o)[2], 1.0);
                c.setRHSConstant(base.getPseudo_oRel_qs().get(on)[7]);
                executor.addConstraint(c);

            }


        }
        //(4)b->t
        c = new GurobiConstraint();
        c.setName(nickname + "_b->t.2_o");
        c.addToLHS(vpRelObstaclesD_qs.get(o)[3], 1.0);
        c.setSense('<');
        c.addToRHS(vp.relD_qs.get(o)[3], 1.0);
        executor.addConstraint(c);
        for (Obstacle on : o.get_dTObstacles()) {
            //(4)b->t.1_o
            c = new GurobiConstraint();
            c.setName(nickname + "_b->t.1_o");
            c.addToLHS(vpRelObstaclesD_qs.get(o)[3], 1.0);
            c.setSense('>');
            c.addToRHS(vp.relD_qs.get(o)[3], 1.0);
            c.setRHSConstant(base.getPseudo_oRel_qs().get(on)[6] - 1.0);
            executor.addConstraint(c);

            if (!o.getName().equals(on.getName())) {
                //(4)b->t.1_on
                c = new GurobiConstraint();
                c.setName(nickname + "_b->t.1_on");
                c.addToLHS(vpRelObstaclesD_qs.get(on)[3], 1.0);
                c.setSense('>');
                c.addToRHS(vp.relD_qs.get(o)[3], 1.0);
                c.setRHSConstant(base.getPseudo_oRel_qs().get(on)[6] - 1.0);
                executor.addConstraint(c);

                //(4)b->t.2_on
                c = new GurobiConstraint();
                c.setName(nickname + "_b->t.2_on");
                c.addToLHS(vpRelObstaclesD_qs.get(on)[3], 2.0);
                c.setSense('<');
                c.addToRHS(vp.relD_qs.get(o)[3], 1.0);
                c.setRHSConstant(base.getPseudo_oRel_qs().get(on)[6]);
                executor.addConstraint(c);

            }
        }


        //Opposite Relation Recognition: aux.1
        c = new GurobiConstraint();
        c.setName("aux.1");
        c.addToLHS(vp.relObstacles_qs.get(o)[0], 1.0);
        c.addToLHS(vp.relObstacles_qs.get(o)[1], 1.0);
        c.addToLHS(vp.relObstacles_qs.get(o)[2], 1.0);
        c.addToLHS(vp.relObstacles_qs.get(o)[3], 1.0);
        c.addToLHS(vp.relObstaclesD_qs.get(o)[0], 1.0);
        c.addToLHS(vp.relObstaclesD_qs.get(o)[1], 1.0);
        c.addToLHS(vp.relObstaclesD_qs.get(o)[2], 1.0);
        c.addToLHS(vp.relObstaclesD_qs.get(o)[3], 1.0);
        c.setSense('<');
        c.addToRHS(vp.relObstacles_q.get(o), 8.0);
        executor.addConstraint(c);
    }


    /**
     * Give the distance regarding 45-degree plane between vi and base
     * @param x vi.x
     * @param y vi.y
     * @param base slave or master
     * @param nickName name
     * @param aux_dist_iqs store the result
     * @param auxQ_dist to select for min{}
     * @throws GRBException Gurobi
     */
    private void auxiliaryDistCons(String nickName, GurobiVariable x, GurobiVariable y, PseudoBase base, GurobiVariable[] aux_dist_iqs, GurobiVariable auxQ_dist) throws GRBException {
        GurobiConstraint c;
        //aux
        c = new GurobiConstraint();
        c.setName(nickName + "_aux_X");
        c.addToLHS(x, 1.0);
        c.setLHSConstant(-base.getX());
        c.setSense('=');
        c.addToRHS(aux_dist_iqs[4], 1.0);
        executor.addConstraint(c);
        executor.addGenConstraintAbs(aux_dist_iqs[0], aux_dist_iqs[4], nickName + "->" + base.getName() + "_absX");
        c = new GurobiConstraint();
        c.setName(nickName + "_aux_Y");
        c.addToLHS(y, 1.0);
        c.setLHSConstant(-base.getY());
        c.setSense('=');
        c.addToRHS(aux_dist_iqs[5], 1.0);
        executor.addConstraint(c);
        executor.addGenConstraintAbs(aux_dist_iqs[1], aux_dist_iqs[5], nickName + "->" + base.getName() + "_absY");
        c = new GurobiConstraint();
        c.setName(nickName + "_aux_XY");
        c.addToLHS(aux_dist_iqs[0], 1.0);
        c.addToLHS(aux_dist_iqs[1], -1.0);
        c.setSense('=');
        c.addToRHS(aux_dist_iqs[6], 1.0);
        executor.addConstraint(c);
        executor.addGenConstraintAbs(aux_dist_iqs[3], aux_dist_iqs[6], nickName + "->" + base.getName() + "_absXY");
        //Min_linear
        c = new GurobiConstraint();
        c.addToLHS(aux_dist_iqs[2], 1.0);
        c.setSense('>');
        c.addToRHS(aux_dist_iqs[0], 1.0);
        c.addToRHS(auxQ_dist, M);
        c.setRHSConstant(-M);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.addToLHS(aux_dist_iqs[2], 1.0);
        c.setSense('>');
        c.addToRHS(aux_dist_iqs[1], 1.0);
        c.addToRHS(auxQ_dist, -M);
        executor.addConstraint(c);
    }

    /**
     * Give the distance regarding 45-degree plane between vi and vi+1
     * @param x1 vi.x
     * @param x2 vi+1.x
     * @param y1 vi.y
     * @param y2 vi+1.y
     * @param nickName name
     * @param aux_dist_iqs store the result
     * @param auxQ_dist to select for min{}
     * @throws GRBException
     */
    private void auxiliaryDistCons(String nickName, GurobiVariable x1, GurobiVariable x2, GurobiVariable y1, GurobiVariable y2, GurobiVariable[] aux_dist_iqs, GurobiVariable auxQ_dist) throws GRBException {
        GurobiConstraint c;

        c = new GurobiConstraint();
        c.setName(nickName + "_aux_x");
        c.addToLHS(x1, 1.0);
        c.addToLHS(x2, -1.0);
        c.setSense('=');
        c.addToRHS(aux_dist_iqs[4], 1.0);
        executor.addConstraint(c);
        executor.addGenConstraintAbs(aux_dist_iqs[0], aux_dist_iqs[4], nickName + "absX");
        c = new GurobiConstraint();
        c.setName(nickName + "_aux_Y");
        c.addToLHS(y1, 1.0);
        c.addToLHS(y2, -1.0);
        c.setSense('=');
        c.addToRHS(aux_dist_iqs[5], 1.0);
        executor.addConstraint(c);
        executor.addGenConstraintAbs(aux_dist_iqs[1], aux_dist_iqs[5], nickName + "absY");
        //Min
        c = new GurobiConstraint();
        c.addToLHS(aux_dist_iqs[2], 1.0);
        c.setSense('>');
        c.addToRHS(aux_dist_iqs[0], 1.0);
        c.addToRHS(auxQ_dist, M);
        c.setRHSConstant(-M);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.addToLHS(aux_dist_iqs[2], 1.0);
        c.setSense('>');
        c.addToRHS(aux_dist_iqs[1], 1.0);
        c.addToRHS(auxQ_dist, -M);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName(nickName + "_aux_XY");
        c.addToLHS(aux_dist_iqs[0], 1.0);
        c.addToLHS(aux_dist_iqs[1], -1.0);
        c.setSense('=');
        c.addToRHS(aux_dist_iqs[6], 1.0);
        executor.addConstraint(c);
        executor.addGenConstraintAbs(aux_dist_iqs[3], aux_dist_iqs[6], nickName + "abs_XY");
    }
    private void auxiliaryDistCons(String nickName, GurobiVariable[] cor1, GurobiVariable[] cor2, GurobiVariable[] aux_dist_iqs, GurobiVariable auxQ_dist) throws GRBException {
        GurobiConstraint c;

        GurobiVariable x1 = cor1[0];
        GurobiVariable y1 = cor1[1];
        GurobiVariable x2 = cor2[0];
        GurobiVariable y2 = cor2[1];

        c = new GurobiConstraint();
        c.setName(nickName + "_aux_x");
        c.addToLHS(x1, 1.0);
        c.addToLHS(x2, -1.0);
        c.setSense('=');
        c.addToRHS(aux_dist_iqs[4], 1.0);
        executor.addConstraint(c);
        executor.addGenConstraintAbs(aux_dist_iqs[0], aux_dist_iqs[4], nickName + "absX");
        c = new GurobiConstraint();
        c.setName(nickName + "_aux_Y");
        c.addToLHS(y1, 1.0);
        c.addToLHS(y2, -1.0);
        c.setSense('=');
        c.addToRHS(aux_dist_iqs[5], 1.0);
        executor.addConstraint(c);
        executor.addGenConstraintAbs(aux_dist_iqs[1], aux_dist_iqs[5], nickName + "absY");
        //Min
        c = new GurobiConstraint();
        c.addToLHS(aux_dist_iqs[2], 1.0);
        c.setSense('>');
        c.addToRHS(aux_dist_iqs[0], 1.0);
        c.addToRHS(auxQ_dist, M);
        c.setRHSConstant(-M);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.addToLHS(aux_dist_iqs[2], 1.0);
        c.setSense('>');
        c.addToRHS(aux_dist_iqs[1], 1.0);
        c.addToRHS(auxQ_dist, -M);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName(nickName + "_aux_XY");
        c.addToLHS(aux_dist_iqs[0], 1.0);
        c.addToLHS(aux_dist_iqs[1], -1.0);
        c.setSense('=');
        c.addToRHS(aux_dist_iqs[6], 1.0);
        executor.addConstraint(c);
        executor.addGenConstraintAbs(aux_dist_iqs[3], aux_dist_iqs[6], nickName + "abs_XY");
    }


    /**
     * Constraints
     * 1) Non-overlapping
     * 2) orientations of virtual points regarding each obstacle: tL, tR, bL, bR, L, R, T, B
     * @param minDist
     * @param vp
     * @param o
     */
    private void buildCons_nonOverlapAndOppositeRelation(int minDist, OctVirtualPointFixSlaveVar vp, Obstacle o) {
        GurobiConstraint c;
        //nonOverlapping
        c = new GurobiConstraint();
        c.setName("nonl");
        c.addToLHS(vp.non_qs.get(o)[0], 1.0);
        c.addToLHS(vp.non_qs.get(o)[1], 1.0);
        c.addToLHS(vp.non_qs.get(o)[2], 1.0);
        c.addToLHS(vp.non_qs.get(o)[3], 1.0);
        c.setSense('<');
        c.setRHSConstant(3.0);
        executor.addConstraint(c);
        //nonl.l
        c = new GurobiConstraint();
        c.setName("nonl.l_1");
        c.addToLHS(vp.x, 1.0);
        c.setSense('>');
        c.addToRHS(vp.non_qs.get(o)[0], M);
        c.setRHSConstant(o.getMinX() - M);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName("nonl.l_2");
        c.addToLHS(vp.x, 1.0);
        c.setSense('<');
        c.addToRHS(vp.non_qs.get(o)[0], M);
        c.setRHSConstant(o.getMinX() - minDist);
        executor.addConstraint(c);
        //nonl.r
        c = new GurobiConstraint();
        c.setName("nonl.r_1");
        c.addToLHS(vp.x, 1.0);
        c.setSense('<');
        c.addToRHS(vp.non_qs.get(o)[1], -M);
        c.setRHSConstant(o.getMaxX() + M);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName("nonl.r_2");
        c.addToLHS(vp.x, 1.0);
        c.setSense('>');
        c.addToRHS(vp.non_qs.get(o)[1], -M);
        c.setRHSConstant(o.getMaxX() + minDist);
        executor.addConstraint(c);
        //nonl.t
        c = new GurobiConstraint();
        c.setName("nonl.t_1");
        c.addToLHS(vp.y, 1.0);
        c.setSense('<');
        c.addToRHS(vp.non_qs.get(o)[2], -M);
        c.setRHSConstant(o.getMaxY() + M);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName("nonl.t_2");
        c.addToLHS(vp.y, 1.0);
        c.setSense('>');
        c.addToRHS(vp.non_qs.get(o)[2], -M);
        c.setRHSConstant(o.getMaxY() + minDist);
        executor.addConstraint(c);
        //nonl.b
        c = new GurobiConstraint();
        c.setName("nonl.b_1");
        c.addToLHS(vp.y, 1.0);
        c.setSense('>');
        c.addToRHS(vp.non_qs.get(o)[3], M);
        c.setRHSConstant(o.getMinY() - M);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName("nonl.b_2");
        c.addToLHS(vp.y, 1.0);
        c.setSense('<');
        c.addToRHS(vp.non_qs.get(o)[3], M);
        c.setRHSConstant(o.getMinY() - minDist);
        executor.addConstraint(c);


        //diagonal
        //rel.ul
        c = new GurobiConstraint();
        c.setName("rel.ul_1");
        c.addToLHS(vp.y, 1.0);
        c.setSense('>');
        c.addToRHS(vp.x, 1.0);
        c.addToRHS(vp.dir_qs.get(o)[0], M);
        c.setRHSConstant(o.getMaxY() - o.getMinX() + minDist - M);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName("rel.ul_2");
        c.addToLHS(vp.y, 1.0);
        c.setSense('<');
        c.addToRHS(vp.x, 1.0);
        c.addToRHS(vp.dir_qs.get(o)[0], M);
        c.setRHSConstant(o.getMaxY() - o.getMinX());
        executor.addConstraint(c);

        //rel.ur
        c = new GurobiConstraint();
        c.setName("rel.ur_1");
        c.addToLHS(vp.y, 1.0);
        c.setSense('>');
        c.addToRHS(vp.x, -1.0);
        c.addToRHS(vp.dir_qs.get(o)[1], M);
        c.setRHSConstant(o.getMaxY() + o.getMaxX() + minDist - M);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName("rel.ur_2");
        c.addToLHS(vp.y, 1.0);
        c.setSense('<');
        c.addToRHS(vp.x, -1.0);
        c.addToRHS(vp.dir_qs.get(o)[1], M);
        c.setRHSConstant(o.getMaxY() + o.getMaxX());
        executor.addConstraint(c);

        //rel.lr
        c = new GurobiConstraint();
        c.setName("rel.lr_1");
        c.addToLHS(vp.y, 1.0);
        c.setSense('>');
        c.addToRHS(vp.x, 1.0);
        c.addToRHS(vp.dir_qs.get(o)[2], -M);
        c.setRHSConstant(o.getMinY() - o.getMaxX());
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName("rel.lr_2");
        c.addToLHS(vp.y, 1.0);
        c.setSense('<');
        c.addToRHS(vp.x, 1.0);
        c.addToRHS(vp.dir_qs.get(o)[2], -M);
        c.setRHSConstant(o.getMinY() - o.getMaxX() - minDist + M);
        executor.addConstraint(c);

        //rel.ll
        c = new GurobiConstraint();
        c.setName("rel.ll_1");
        c.addToLHS(vp.y, 1.0);
        c.setSense('>');
        c.addToRHS(vp.x, -1.0);
        c.addToRHS(vp.dir_qs.get(o)[3], -M);
        c.setRHSConstant(o.getMinY() + o.getMinX());
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName("rel.ll_2");
        c.addToLHS(vp.y, 1.0);
        c.setSense('<');
        c.addToRHS(vp.x, -1.0);
        c.addToRHS(vp.dir_qs.get(o)[3], -M);
        c.setRHSConstant(o.getMinY() + o.getMinX() - minDist + M);
        executor.addConstraint(c);


        //tL
        allZeroBVars("tL", vp.rel_qs.get(o)[0], new ArrayList<>(Arrays.asList(vp.dir_qs.get(o)[1], vp.dir_qs.get(o)[3], vp.auxRel_qs.get(o)[0])));
        //nonL * nonT
        linearizeProduct2BVars(vp.auxRel_qs.get(o)[0], vp.non_qs.get(o)[0], vp.non_qs.get(o)[2]);


        //tR
        allZeroBVars("tR", vp.rel_qs.get(o)[1], new ArrayList<>(Arrays.asList(vp.dir_qs.get(o)[0], vp.dir_qs.get(o)[2], vp.auxRel_qs.get(o)[1])));
        //nonR * nonT
        linearizeProduct2BVars(vp.auxRel_qs.get(o)[1], vp.non_qs.get(o)[1], vp.non_qs.get(o)[2]);

        //bL
        allZeroBVars("bL", vp.rel_qs.get(o)[2], new ArrayList<>(Arrays.asList(vp.dir_qs.get(o)[0], vp.dir_qs.get(o)[2], vp.auxRel_qs.get(o)[2])));
        //nonL * nonB
        linearizeProduct2BVars(vp.auxRel_qs.get(o)[2], vp.non_qs.get(o)[0], vp.non_qs.get(o)[3]);


        //bR
        allZeroBVars("bR", vp.rel_qs.get(o)[3], new ArrayList<>(Arrays.asList(vp.dir_qs.get(o)[1], vp.dir_qs.get(o)[3], vp.auxRel_qs.get(o)[3])));
        //nonR * nonB
        linearizeProduct2BVars(vp.auxRel_qs.get(o)[3], vp.non_qs.get(o)[1], vp.non_qs.get(o)[3]);


        //Directly direction
        //L
        allOneBVars("dL", vp.relD_qs.get(o)[0], new ArrayList<>(Arrays.asList(vp.non_qs.get(o)[1], vp.non_qs.get(o)[2], vp.non_qs.get(o)[3])));

        //R
        allOneBVars("dR", vp.relD_qs.get(o)[1], new ArrayList<>(Arrays.asList(vp.non_qs.get(o)[0], vp.non_qs.get(o)[2], vp.non_qs.get(o)[3])));


        //T
        allOneBVars("dT", vp.relD_qs.get(o)[2], new ArrayList<>(Arrays.asList(vp.non_qs.get(o)[0], vp.non_qs.get(o)[1], vp.non_qs.get(o)[3])));

        //B
       allOneBVars("dB", vp.relD_qs.get(o)[3], new ArrayList<>(Arrays.asList(vp.non_qs.get(o)[0], vp.non_qs.get(o)[1], vp.non_qs.get(o)[2])));

    }







}

