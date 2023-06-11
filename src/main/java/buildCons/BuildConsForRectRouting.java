package buildCons;

import grb.GurobiConstraint;
import grb.GurobiExecutor;
import grb.GurobiVariable;
import gurobi.GRBException;
import shapeVar.RectFirstVirtualPointVar;
import shapeVar.RectVirtualPointVar;
import shapes.Obstacle;
import shapes.PseudoBase;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Map;

/**
 * @auther lianmeng
 * @create 10.06.23
 */
public class BuildConsForRectRouting extends BuildCons {

    public ArrayList<RectVirtualPointVar> rectVirtualPointVars;
    public GurobiVariable busLength, branchLength;

    public BuildConsForRectRouting(ArrayList<RectVirtualPointVar> rectVirtualPointVars, ArrayList<Obstacle> obstacles, ArrayList<PseudoBase> slaves, PseudoBase master, GurobiVariable busLength, GurobiVariable branchLength, GurobiExecutor executor, int m, int minDist) {
        super(obstacles, slaves, master, executor, m, minDist);

        this.rectVirtualPointVars = rectVirtualPointVars;
        this.busLength = busLength;
        this.branchLength = branchLength;
    }

    public void buildCons() throws GRBException {
        GurobiConstraint c;
        RectVirtualPointVar vpN;

        //busLength
        GurobiConstraint c_bus = new GurobiConstraint();
        c_bus.setName("c_bus");
        c_bus.addToLHS(busLength, 1.0);
        c_bus.setSense('=');
        executor.addConstraint(c_bus);
        GurobiConstraint c_branch = new GurobiConstraint();
        c_branch.setName("c_branch");
        c_branch.addToLHS(branchLength, 1.0);
        c_branch.setSense('=');
        executor.addConstraint(c_branch);

        //vp->Slave connection
        for (PseudoBase sv : slaves) {
            c = new GurobiConstraint();
            for (RectVirtualPointVar vp : rectVirtualPointVars) {
                c.addToLHS(vp.vsCnn_q.get(sv), 1.0);
            }
            c.setSense('=');
            c.setRHSConstant(1.0);
            executor.addConstraint(c);
        }

        for (RectVirtualPointVar vp : rectVirtualPointVars){
            int i = rectVirtualPointVars.indexOf(vp);
            /*
            Rectilinear Routing: non-overlapping and orientation detection
             */
            for (Obstacle o : obstacles){
                buildCons_nonOverlapAndOppositeRelation(minDist, vp, o);
            }
            /*
            Index 0 -- vps.size() - 1:
            Connection with next virtualPoint
             */
            if (i < rectVirtualPointVars.size() - 1){
                vpN = rectVirtualPointVars.get(i + 1);
                String name = "v" + i + "->v" + (i+1);

                //add To busLength
                c_bus.addToRHS(vp.dist_iq, 1.0);
                /*
                d_i_i+1 without Detour
                VVVV
                 */
                c = new GurobiConstraint();
                c.setName(name + "_pathLength");
                c.addToLHS(vp.dist_iq, 1.0);
                c.setSense('>');
                c.addToRHS(vp.aux_dist_iqs[0], 1.0);
                c.addToRHS(vp.aux_dist_iqs[1], 1.0);
                executor.addConstraint(c);
                auxiliaryManhattanDistCons(name + "_d_woDetour", vp.x, vp.y, vpN.x, vpN.y, vp.aux_dist_iqs);
                /*
                AAAA
                d_i_i+1
                 */

                //d_ij regarding detour
                GurobiConstraint c_pathLength = new GurobiConstraint();
                c_pathLength.setName(name + "_pathLength_detour");
                c_pathLength.addToLHS(vp.dist_iq, 1.0);
                c_pathLength.setSense('>');
                c_pathLength.addToRHS(vp.dOut_iq, 1.0);
                c_pathLength.addToRHS(vp.dIn_iq, 1.0);
                c_pathLength.addToRHS(vp.detour_q, M);
                c_pathLength.setRHSConstant(-M);
                executor.addConstraint(c_pathLength);

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
                    Rectilinear Routing: Opposite Relations Detection between vi and vi+1
                     */

                    buildCons_oppositeRelations(oName, vp, vpN, o);

                    /*
                    CornerRules
                     */
                    c = new GurobiConstraint();
                    c.setName(oName + "_cornerRules1");
                    c.addToLHS(vp.corner_qs.get(o)[0], 1.0);
                    c.addToLHS(vp.corner_qs.get(o)[1], 1.0);
                    c.setSense('=');
                    c.addToRHS(vp.relObstacles_q.get(o), 1.0);
                    executor.addConstraint(c);

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
                            c.setName(onName + "_pathLength.m->n");
                            c.addToLHS(vp.dOmOn_iq.get(o).get(on), 1.0);
                            c.setSense('>');
                            c.addToRHS(vp.aux_dOmOn_iqs.get(o).get(on)[0], 1.0);
                            c.addToRHS(vp.aux_dOmOn_iqs.get(o).get(on)[1], 1.0);
                            c.addToRHS(vp.omOnCnn_q.get(o).get(on), M);
                            c.setRHSConstant(-M);
                            executor.addConstraint(c);
                            auxiliaryManhattanDistCons(onName + "_pathLength.m->n", vp.oCoordinate_iqs.get(o), vp.oCoordinate_iqs.get(on), vp.aux_dOmOn_iqs.get(o).get(on));

                            c_pathLength.addToRHS(vp.dOmOn_iq.get(o).get(on), 1.0);
                        }
                    }
                    //coordinate
                    buildCons_oCoordinate(oName + "_", o, vp.oCoordinate_iqs.get(o), vp.corner_qs.get(o), vp.relObstacles_q.get(o));

                    //pathLength.stp->
                    c = new GurobiConstraint();
                    c.setName(oName + "_pathLength.stp->");
                    c.addToLHS(vp.dOut_iq, 1.0);
                    c.setSense('>');
                    c.addToRHS(vp.aux_dOut_iqs.get(o)[0], 1.0);
                    c.addToRHS(vp.aux_dOut_iqs.get(o)[1], 1.0);
                    c.addToRHS(vp.startEndObstacles_qs.get(o)[0], M);
                    c.setRHSConstant(-M);
                    executor.addConstraint(c);
                    auxiliaryManhattanDistCons(oName + "_pathLength.stp->", vp.x, vp.y, vp.oCoordinate_iqs.get(o)[0], vp.oCoordinate_iqs.get(o)[1], vp.aux_dOut_iqs.get(o));


                    //pathLength.->Evp
                    c = new GurobiConstraint();
                    c.setName(oName + "_pathLength.->Evp.min");
                    c.addToLHS(vp.dIn_iq, 1.0);
                    c.setSense('>');
                    c.addToRHS(vp.aux_dIn_iqs.get(o)[0], 1.0);
                    c.addToRHS(vp.aux_dIn_iqs.get(o)[1], 1.0);
                    c.addToRHS(vp.startEndObstacles_qs.get(o)[1], M);
                    c.setRHSConstant(-M);
                    executor.addConstraint(c);
                    auxiliaryManhattanDistCons(oName + "_pathLength.->Evp", vp.oCoordinate_iqs.get(o)[0], vp.oCoordinate_iqs.get(o)[1], vpN.x, vpN.y, vp.aux_dIn_iqs.get(o));


                }

            }


            /*
            vp -> slave
             */
            /*
            Connection with Slaves
             */
            GurobiConstraint c_vsCnn = new GurobiConstraint();
            c_vsCnn.setSense('=');
            c_vsCnn.setRHSConstant(1.0);
            executor.addConstraint(c_vsCnn);
            for (PseudoBase sv : slaves){

                //sv.1
                c_vsCnn.addToLHS(vp.vsCnn_q.get(sv), 1.0);
                //add to branch length:
                c_branch.addToRHS(vp.corDist_iq, 1.0);
                //sv.3_geq
                c = new GurobiConstraint();
                c.setName("sv.3_geq:Min");
                c.addToLHS(vp.corDist_iq, 1.0);
                c.setSense('>');
                c.addToRHS(vp.vs_dist_iq.get(sv), 1.0);
                c.addToRHS(vp.vsCnn_q.get(sv), M);
                c.setRHSConstant(-M);
                executor.addConstraint(c);

                String sName = "v" + i +  sv.getName();

                buildCons_vpToBase(sName, vp, sv, vp.vs_dist_iq.get(sv), vp.aux_vsDist_iqs.get(sv), vp.vs_dOut_iq.get(sv), vp.vs_dIn_iq.get(sv), vp.vs_detour_q.get(sv), vp.vs_relObstacles_q.get(sv), vp.linAux_vsInOut_qs.get(sv), vp.vs_startEndObstacles_qs.get(sv), vp.vs_relObstaclesD_qs.get(sv), vp.vs_corner_qs.get(sv), vp.vs_omOnCnn_q.get(sv), vp.linAux_vsOmOn_qs.get(sv), vp.vs_dOmOn_iq.get(sv), vp.aux_vsdOmOn_iqs.get(sv), vp.vs_oCoordinate_iqs.get(sv), vp.aux_vsdOut_iqs.get(sv), vp.aux_vsdIn_iqs.get(sv));


            }
            /*
            v0 -> master
             */
            if (vp instanceof RectFirstVirtualPointVar){
                String mName = "vm";
                RectFirstVirtualPointVar vF = (RectFirstVirtualPointVar) vp;
                //add to busLength
                c_bus.addToRHS(vF.vm_dist_iq, 1.0);

                buildCons_vpToBase(mName, vF, master, vF.vm_dist_iq, vF.aux_vmDist_iqs, vF.vm_dOut_iq, vF.vm_dIn_iq, vF.vm_detour_q, vF.vm_relObstacles_q, vF.linAux_vmInOut_qs, vF.vm_startEndObstacles_qs, vF.vm_relObstaclesD_qs, vF.vm_corner_qs, vF.vm_omOnCnn_q, vF.linAux_vmOmOn_qs, vF.vm_dOmOn_iq, vF.aux_vmdOmOn_iqs, vF.vm_oCoordinate_iqs, vF.aux_vmdOut_iqs, vF.aux_vmdIn_iqs);

            }

        }



    }

    private void buildCons_vpToBase(String nickname, RectVirtualPointVar vp, PseudoBase base, GurobiVariable dist_iq, GurobiVariable[] aux_dist_iqs, GurobiVariable dOut_iq, GurobiVariable dIn_iq, GurobiVariable detour_q, Map<Obstacle, GurobiVariable> relObstacles_q, Map<Obstacle, GurobiVariable[]> linAux_InOut_qs, Map<Obstacle, GurobiVariable[]> startEndObstacles_qs, Map<Obstacle, GurobiVariable[]> relObstaclesD_qs, Map<Obstacle, GurobiVariable[]> corner_qs, Map<Obstacle, Map<Obstacle, GurobiVariable>> omOnCnn_q, Map<Obstacle, Map<Obstacle, GurobiVariable[]>> linAux_OmOn_qs, Map<Obstacle, Map<Obstacle, GurobiVariable>> dOmOn_iq, Map<Obstacle, Map<Obstacle, GurobiVariable[]>> aux_dOmOn_iqs, Map<Obstacle, GurobiVariable[]> oCoordinate_iqs, Map<Obstacle, GurobiVariable[]> aux_dOut_iqs, Map<Obstacle, GurobiVariable[]> aux_dIn_iqs) throws GRBException {
        GurobiConstraint c;
    /*
    d_i_Base without detour
    VVVV
     */
        c = new GurobiConstraint();
        c.setName(nickname + "_pathLength");
        c.addToLHS(dist_iq, 1.0);
        c.setSense('>');
        c.addToRHS(aux_dist_iqs[0], 1.0);
        c.addToRHS(aux_dist_iqs[1], 1.0);
        executor.addConstraint(c);
        auxiliaryManhattanDistCons(nickname + "_d_woDetour", vp.x, vp.y, base, aux_dist_iqs);
                /*
                AAAA
                d_i_Base
                 */
        //d_i_Base regarding detour
        GurobiConstraint c_base_pathLength = new GurobiConstraint();
        c_base_pathLength.setName(nickname + "_pathLength_detour_min");
        c_base_pathLength.addToLHS(dist_iq, 1.0);
        c_base_pathLength.setSense('>');
        c_base_pathLength.addToRHS(dOut_iq, 1.0);
        c_base_pathLength.addToRHS(dIn_iq, 1.0);
        c_base_pathLength.addToRHS(detour_q, M);
        c_base_pathLength.setRHSConstant(-M);
        executor.addConstraint(c_base_pathLength);

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
            linearizeProduct2BVars(linAux_InOut_qs.get(o)[0], startEndObstacles_qs.get(o)[0], relObstacles_q.get(o));
            c_base_svpToObstacle.addToLHS(linAux_InOut_qs.get(o)[0], 1.0);

            //vp->Base: cnnRules.6_lin
            linearizeProduct2BVars(linAux_InOut_qs.get(o)[1], startEndObstacles_qs.get(o)[1], relObstacles_q.get(o));
            c_base_obstacleToEvp.addToLHS(linAux_InOut_qs.get(o)[1], 1.0);

            /*
            vp->Base: opposite relations detection between vi and si
             */
            buildCons_oppositeRelations(oNickname, vp, base, o, relObstaclesD_qs);
            /*
            vp->Base: cornerRules
             */
            c = new GurobiConstraint();
            c.setName(oNickname + "_cornerRules1");
            c.addToLHS(corner_qs.get(o)[0], 1.0);
            c.addToLHS(corner_qs.get(o)[1], 1.0);
            c.setSense('=');
            c.addToRHS(relObstacles_q.get(o), 1.0);
            executor.addConstraint(c);

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
            for (Obstacle on : obstacles) {
                if (!o.getName().equals(on.getName())) {
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
                    c.setName(sonName + "_pathLength.m->n");
                    c.addToLHS(dOmOn_iq.get(o).get(on), 1.0);
                    c.setSense('>');
                    c.addToRHS(aux_dOmOn_iqs.get(o).get(on)[0], 1.0);
                    c.addToRHS(aux_dOmOn_iqs.get(o).get(on)[1], 1.0);
                    c.addToRHS(omOnCnn_q.get(o).get(on), M);
                    c.setRHSConstant(-M);
                    executor.addConstraint(c);
                    auxiliaryManhattanDistCons(sonName + "_pathLength.m->n", oCoordinate_iqs.get(o), oCoordinate_iqs.get(on), aux_dOmOn_iqs.get(o).get(on));

                    c_base_pathLength.addToRHS(dOmOn_iq.get(o).get(on), 1.0);

                }
            }

            //vp->Base: coordinate
            buildCons_oCoordinate(oNickname + "_", o, oCoordinate_iqs.get(o), corner_qs.get(o), relObstacles_q.get(o));

            //vp->Base: pathLength.stp->
            c = new GurobiConstraint();
            c.setName(oNickname + "_pathLength.stp->");
            c.addToLHS(dOut_iq, 1.0);
            c.setSense('>');
            c.addToRHS(aux_dOut_iqs.get(o)[0], 1.0);
            c.addToRHS(aux_dOut_iqs.get(o)[1], 1.0);
            c.addToRHS(startEndObstacles_qs.get(o)[0], M);
            c.setRHSConstant(-M);
            executor.addConstraint(c);
            auxiliaryManhattanDistCons(oNickname + "_pathLength.stp->", vp.x, vp.y, oCoordinate_iqs.get(o)[0], oCoordinate_iqs.get(o)[1], aux_dOut_iqs.get(o));

            //vp->Base: pathLength.->Evp
            c = new GurobiConstraint();
            c.setName(oNickname + "_pathLength.->Evp");
            c.addToLHS(dIn_iq, 1.0);
            c.setSense('>');
            c.addToRHS(aux_dIn_iqs.get(o)[0], 1.0);
            c.addToRHS(aux_dIn_iqs.get(o)[1], 1.0);
            c.addToRHS(startEndObstacles_qs.get(o)[1], M);
            c.setRHSConstant(-M);
            executor.addConstraint(c);
            auxiliaryManhattanDistCons(oNickname + "_pathLength.->Evp", oCoordinate_iqs.get(o)[0], oCoordinate_iqs.get(o)[1], base, aux_dIn_iqs.get(o));
        }
    }

    /**
     * Rectlinear Routing: Coordinates of the enter and leave corners
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
        c.addToRHS(corner_qs[1], o.getMaxX());
        c.addToRHS(relObstacles_q, -M);
        c.setRHSConstant(M);
        c = new GurobiConstraint();
        c.setName(nickName + "Cor.x_geq");
        c.addToLHS(coordinates[0], 1.0);
        c.setSense('>');
        c.addToRHS(corner_qs[0], o.getMinX());
        c.addToRHS(corner_qs[1], o.getMaxX());
        c.addToRHS(relObstacles_q, M);
        c.setRHSConstant(-M);
        c = new GurobiConstraint();
        c.setName(nickName + "Cor.y_leq");
        c.addToLHS(coordinates[1], 1.0);
        c.setSense('<');
        c.addToRHS(corner_qs[0], o.getMinY());
        c.addToRHS(corner_qs[1], o.getMaxY());
        c.addToRHS(relObstacles_q, -M);
        c.setRHSConstant(M);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName(nickName + "Cor.y_geq");
        c.addToLHS(coordinates[1], 1.0);
        c.setSense('>');
        c.addToRHS(corner_qs[0], o.getMinY());
        c.addToRHS(corner_qs[1], o.getMaxY());
        c.addToRHS(relObstacles_q, M);
        c.setRHSConstant(-M);
        executor.addConstraint(c);
    }



    /**
     * Rectilinear Routing: Constraints for opposite relations between vi and vi+1
     * @param nickname name
     * @param vp vi
     * @param vpN vi+1
     * @param o obstacle
     */
    private void buildCons_oppositeRelations(String nickname, RectVirtualPointVar vp, RectVirtualPointVar vpN, Obstacle o) {
        GurobiConstraint c;

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
        c.addToLHS(vp.relObstaclesD_qs.get(o)[0], 1.0);
        c.addToLHS(vp.relObstaclesD_qs.get(o)[1], 1.0);
        c.addToLHS(vp.relObstaclesD_qs.get(o)[2], 1.0);
        c.addToLHS(vp.relObstaclesD_qs.get(o)[3], 1.0);
        c.setSense('<');
        c.addToRHS(vp.relObstacles_q.get(o), 4.0);
        executor.addConstraint(c);
    }
    private void buildCons_oppositeRelations(String nickname, RectVirtualPointVar vp, PseudoBase base, Obstacle o, Map<Obstacle, GurobiVariable[]> vpRelObstaclesD_qs) {
        GurobiConstraint c;

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
        c.addToLHS(vp.relObstaclesD_qs.get(o)[0], 1.0);
        c.addToLHS(vp.relObstaclesD_qs.get(o)[1], 1.0);
        c.addToLHS(vp.relObstaclesD_qs.get(o)[2], 1.0);
        c.addToLHS(vp.relObstaclesD_qs.get(o)[3], 1.0);
        c.setSense('<');
        c.addToRHS(vp.relObstacles_q.get(o), 4.0);
        executor.addConstraint(c);
    }


    /**
     * Constraints
     * 1) Non-overlapping
     * 2) orientations of virtual points regarding each obstacle: L, R, T, B
     * @param minDist
     * @param vp
     * @param o
     */
    private void buildCons_nonOverlapAndOppositeRelation(int minDist, RectVirtualPointVar vp, Obstacle o) {
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

    /**
     * Give the distance regarding Manhattan geometry between enter and leave corners
     * @param nickName
     * @param x1
     * @param y1
     * @param base
     * @param aux_dist_iqs
     * @throws GRBException
     */
    private void auxiliaryManhattanDistCons(String nickName, GurobiVariable x1, GurobiVariable y1, PseudoBase base, GurobiVariable[] aux_dist_iqs) throws GRBException{
        GurobiConstraint c;

        c = new GurobiConstraint();
        c.setName(nickName + "_aux_x");
        c.addToLHS(x1, 1.0);
        c.setLHSConstant(-base.getX());
        c.setSense('=');
        c.addToRHS(aux_dist_iqs[2], 1.0);
        executor.addConstraint(c);
        executor.addGenConstraintAbs(aux_dist_iqs[0], aux_dist_iqs[2], nickName + "absX");
        c = new GurobiConstraint();
        c.setName(nickName + "_aux_Y");
        c.addToLHS(y1, 1.0);
        c.setLHSConstant(-base.getY());
        c.setSense('=');
        c.addToRHS(aux_dist_iqs[3], 1.0);
        executor.addConstraint(c);
        executor.addGenConstraintAbs(aux_dist_iqs[1], aux_dist_iqs[3], nickName + "absY");
    }


}
