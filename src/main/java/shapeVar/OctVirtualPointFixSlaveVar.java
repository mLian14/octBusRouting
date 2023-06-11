package shapeVar;

import grb.GurobiVariable;
import shapes.Obstacle;
import shapes.PseudoBase;

import java.util.HashMap;
import java.util.Map;

/**
 * @auther lianmeng
 * @create 09.04.23
 */
public class OctVirtualPointFixSlaveVar {
    public PseudoBase slave;

    public GurobiVariable x;//x-coordinate of v_point
    public GurobiVariable y;//y-coordinate of v_point


    /*
    Detour triggering regarding NEXT VirtualPoint
     */

    /*
    Non-overlapping
    0: nonL
    1: nonR
    2: nonT
    3: nonB
     */
    public Map<Obstacle, GurobiVariable[]> non_qs;//binary variables for non-overlapping

    /*
    dir_qs
    0: ul
    1: ur
    2: lr
    3: ll
     */
    public Map<Obstacle, GurobiVariable[]> dir_qs;//binary variables for (rel.ul) -- (rel.u)

    /*
    rel_qs
    0: o_tL
    1: o_tR
    2: o_bL
    3: o_bR
     */
    public Map<Obstacle, GurobiVariable[]> rel_qs;//binary variables for (tL) -- (bR)
    /*
    0: nonL * nonT
    1: nonR * nonT
    2: nonL * nonB
    3: nonR * nonB
     */
    public Map<Obstacle, GurobiVariable[]> auxRel_qs;
    /*
    relD_qs
    0: o_L
    1: o_R
    2: o_T
    3: o_B
     */
    public Map<Obstacle, GurobiVariable[]> relD_qs;//binary variables for (L) (R) (T) (B)
    /*
    relObstacles_qs
    0: ul->lr
    1: lr->ul
    2: ur->ll
    3: ll->ur
     */
    public Map<Obstacle, GurobiVariable[]> relObstacles_qs;//binary variables for (ul->lr.1) -- (ll->ur.1)

    /*
    relObstaclesD_qs
    0: l->r
    1: r->l
    2: t->b
    3: b->t
     */
    public Map<Obstacle, GurobiVariable[]> relObstaclesD_qs;
    /*
    indicate the relevance of each obstacle
     */
    public Map<Obstacle, GurobiVariable> relObstacles_q;

    /*
    detour_qs
    q_ij^d: detour trigger
     */
    public GurobiVariable detour_q;

    /*
    Path Length Computation regarding Next VirtualPoint
     */

    /*
    enterCorner_qs, leaveCorner_qs
    0: ll
    1: ur
    2: ul
    3: lr
     */
    public Map<Obstacle, GurobiVariable[]> enterCorner_qs;
    public Map<Obstacle, GurobiVariable[]> leaveCorner_qs;


    /*
    enterCoordinate_iqs, leaveCoordinate_iqs
    0: x_m
    1: y_m
     */
    public Map<Obstacle, GurobiVariable[]> enterCoordinate_iqs;
    public Map<Obstacle, GurobiVariable[]> leaveCoordinate_iqs;
    /*
    Auxiliary absolute values (Manhattan): aux_dOm_iqs
    Manhattan distance between enter and leave corners
     */
    public Map<Obstacle, GurobiVariable[]> aux_dOm_iqs;

    /*
    startEndObstacle_qs
    0: vi->
    1: ->vi+1
     */
    public Map<Obstacle, GurobiVariable[]> startEndObstacles_qs;//binaryVar regarding next virtualPoint for starting and end point connection

    /*
    dOut_cqs
    0: d_vi->:min
    1: d_vi->:xy
     */
    public GurobiVariable[] dOut_cqs;
    /*
    Auxiliary absolute values: aux_dOut_iqs
    For pathLength.stpTo
     */
    public Map<Obstacle, GurobiVariable[]> aux_dOut_iqs;
    public Map<Obstacle, GurobiVariable> auxQ_dOut;


    /*
    dIn_cqs
    0: d->vj:min
    1: d->vj:xy
     */
    public GurobiVariable[] dIn_cqs;
    /*
    Auxiliary absolute values: aux_dIn_iqs
    For pathLength.toEvp
     */
    public Map<Obstacle, GurobiVariable[]> aux_dIn_iqs;
    public Map<Obstacle, GurobiVariable> auxQ_dIn;

    /*
    linearize Product
    q_->m * om.relO_q (cnnRules.5)
    q_m-> * om.relO_q (cnnRules.6)
     */
    public Map<Obstacle, GurobiVariable[]> linAux_inOut_qs;

    /*
    omOnCnn_q
    q_i_i+1^m->n
     */
    public Map<Obstacle, Map<Obstacle, GurobiVariable>> omOnCnn_q;//binaryVar regarding next virtualPoint for relObstacles' connection

    /*
    dOmOn_cqs
    0: d_m->n: min
    1: d_m->n: xy
     */
    public Map<Obstacle, Map<Obstacle, GurobiVariable[]>> dOmOn_cqs;
    /*
    Auxiliary absolute values: aux_dOmOn_iqs
    For pathLength.mTon
     */
    public Map<Obstacle, Map<Obstacle, GurobiVariable[]>> aux_dOmOn_iqs;
    public Map<Obstacle, Map<Obstacle, GurobiVariable>> auxQ_dOmOn;
    /*
    linearize Product
    q_m->n * on.relO_q (cnnRules.3)
    q_m->n * om.relO_q (cnnRules.4)
     */
    public Map<Obstacle, Map<Obstacle, GurobiVariable[]>> linAux_OmOn_qs;


    /*
    dist_cqs: vv_dist
    0: dvp->vp:min
    1: dvp->vp:xy
     */
    public GurobiVariable[] dist_cqs;
    /*
    Auxiliary absolute values:
    For pathLength vp->vp without detour
     */
    public GurobiVariable[] aux_dist_iqs;
    public GurobiVariable auxQ_dist;






    //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=//
    /*
    Detour triggering regarding Slaves
     */


    /*
    vs_relObstacles_qs
    0: ul->lr
    1: lr->ul
    2: ur->ll
    3: ll->ur
     */
    public Map<Obstacle, GurobiVariable[]> vs_relObstacles_qs;//binary variables for (ul->lr.1) -- (ll->ur.1)

    /*
    vs_relObstaclesD_qs
    0: l->r
    1: r->l
    2: t->b
    3: b->t
     */
    public Map<Obstacle, GurobiVariable[]> vs_relObstaclesD_qs;
    /*
    vp->slave: indicate relevant Obstacles
     */
    public Map<Obstacle, GurobiVariable> vs_relObstacles_q;

    /*
    vs_detour_qs
    q_ij^d: detour trigger
     */
    public GurobiVariable vs_detour_q;//binary variables regarding Slaves for (ul->lr.2) -- (ll->ur.2)



    /*
    Path Length Computation regarding Slaves
     */



    /*
    vs_inOutCnn_qs
    0: vi->
    1: ->si
     */
    public Map<Obstacle, GurobiVariable[]> vs_startEndObstacles_qs;//binaryVar regarding Slaves for starting and end point connection

    /*
    vs_dOut_cqs
    0: vs_dOut: min
    1: vs_dOut: xy
     */
    public GurobiVariable[] vs_dOut_cqs;
    /*
    Auxiliary absolute values: aux_vsdOut_iqs
    For pathLength.svpTo
    0: |vi.x - x_m|
    1: |vi.y - y_m|
    2: min(|vi.x - x_m|, |vi.y - y_m|)
    3: ||vi.x - x_m| - |vi.y - y_m||
    4: vi.x - x_m
    5: vi.y - y_m
    6: |vi.x - x_m| - |vi.y - y_m|
     */
    public Map<Obstacle, GurobiVariable[]> aux_vsdOut_iqs;
    public Map<Obstacle, GurobiVariable> auxQ_vsdOut;

    /*
    vs_dIn_cqs
    For pathLength.toEp
    0: vs_odIn: min
    1: vs_odIn: xy
     */
    public GurobiVariable[] vs_dIn_cqs;
    /*
    Auxiliary absolute values: aux_vsdIn_iqs
     */
    public Map<Obstacle, GurobiVariable[]> aux_vsdIn_iqs;
    public Map<Obstacle, GurobiVariable> auxQ_vsdIn;
    /*
    linearize Product
    q_->m * om.relO_q (cnnRules.5)
    q_m-> * om.relO_q (cnnRules.6)
     */
    public Map<Obstacle, GurobiVariable[]> linAux_vsInOut_qs;


    /*
        vs_enterCorner_qs, vs_leaveCorner_qs
        0: ll
        1: ur
        2: ul
        3: lr
         */
    public Map<Obstacle, GurobiVariable[]> vs_enterCorner_qs;
    public Map<Obstacle, GurobiVariable[]> vs_leaveCorner_qs;

    /*
    vs_omOnCnn_q
    q_i_si^m->n
     */
    public Map<Obstacle, Map<Obstacle, GurobiVariable>> vs_omOnCnn_q;//binaryVar regarding Slaves for relObstacles' connection


    /*
    vs_dOmOn_cqs
    0: vs_d_m->n: min
    1: vs_d_m->n: xy
     */
    public Map<Obstacle, Map<Obstacle, GurobiVariable[]>> vs_dOmOn_cqs;

    /*
    vs_enterCoordinate_iqs, vs_leaveCoordinate_iqs
    0: x_m
    1: y_m
     */
    public Map<Obstacle, GurobiVariable[]> vs_enterCoordinate_iqs;//intVar regarding Slaves: coordinates of the selected intermedia point
    public Map<Obstacle, GurobiVariable[]> vs_leaveCoordinate_iqs;
    /*
    Auxiliary absolute values (Manhattan): aux_vsdOm_iqs
    Manhattan distance between enter and leave corners
     */
    public Map<Obstacle, GurobiVariable[]> aux_vsdOm_iqs;

    /*
    Auxiliary absolute values: aux_vsdOmOn_iqs
    For pathLength.mTon
     */
    public Map<Obstacle, Map<Obstacle, GurobiVariable[]>> aux_vsdOmOn_iqs;
    public Map<Obstacle, Map<Obstacle, GurobiVariable>> auxQ_vsdOmOn;


    /*
    linearize Product
    q_m->n * on.relO_q (cnnRules.3)
    q_m->n * om.relO_q (cnnRules.4)
     */
    public Map<Obstacle, Map<Obstacle, GurobiVariable[]>> linAux_vsOmOn_qs;


    /*
    dvs
    0: dvs: min
    1: dvs: xy
     */
    public GurobiVariable[] vs_dist_cqs;

    /*
    Auxiliary absolute values:
    For pathLength vp->slave without detour
     */
    public GurobiVariable[] aux_vs_dist_iqs;
    public GurobiVariable auxQ_vs_dist;


    public OctVirtualPointFixSlaveVar(PseudoBase slave) {

        this.slave = slave;

        this.non_qs = new HashMap<>();
        this.dir_qs = new HashMap<>();
        this.rel_qs = new HashMap<>();
        this.auxRel_qs = new HashMap<>();
        this.relD_qs = new HashMap<>();
        this.relObstacles_qs = new HashMap<>();
        this.relObstaclesD_qs = new HashMap<>();
        this.relObstacles_q = new HashMap<>();


        this.enterCorner_qs = new HashMap<>();
        this.leaveCorner_qs = new HashMap<>();


        this.omOnCnn_q = new HashMap<>();
        this.enterCoordinate_iqs = new HashMap<>();
        this.leaveCoordinate_iqs = new HashMap<>();
        this.aux_dOm_iqs = new HashMap<>();
        this.startEndObstacles_qs = new HashMap<>();

        this.aux_dOut_iqs = new HashMap<>();
        this.auxQ_dOut = new HashMap<>();

        this.aux_dIn_iqs = new HashMap<>();
        this.auxQ_dIn = new HashMap<>();

        this.linAux_inOut_qs = new HashMap<>();

        this.dOmOn_cqs = new HashMap<>();
        this.aux_dOmOn_iqs = new HashMap<>();
        this.auxQ_dOmOn = new HashMap<>();
        this.linAux_OmOn_qs = new HashMap<>();






        this.vs_relObstacles_qs = new HashMap<>();
        this.vs_relObstaclesD_qs = new HashMap<>();
        this.vs_relObstacles_q = new HashMap<>();

        this.vs_enterCorner_qs = new HashMap<>();
        this.vs_leaveCorner_qs = new HashMap<>();


        this.vs_dOmOn_cqs = new HashMap<>();
        this.vs_enterCoordinate_iqs = new HashMap<>();
        this.vs_leaveCoordinate_iqs = new HashMap<>();
        this.aux_vsdOm_iqs = new HashMap<>();
        this.vs_omOnCnn_q = new HashMap<>();
        this.vs_startEndObstacles_qs = new HashMap<>();

        this.aux_vsdOut_iqs = new HashMap<>();
        this.auxQ_vsdOut = new HashMap<>();
        this.aux_vsdIn_iqs = new HashMap<>();
        this.auxQ_vsdIn = new HashMap<>();
        this.linAux_vsInOut_qs = new HashMap<>();

        this.aux_vsdOmOn_iqs = new HashMap<>();
        this.auxQ_vsdOmOn = new HashMap<>();
        this.linAux_vsOmOn_qs = new HashMap<>();




    }
}
