package shapeVar;

import grb.GurobiVariable;
import shapes.Obstacle;
import shapes.PseudoBase;

import java.util.HashMap;
import java.util.Map;

/**
 * @auther lianmeng
 * @create 10.06.23
 */
public class RectVirtualPointVar {

    public GurobiVariable x;//x-coordinate of v_point
    public GurobiVariable y;//y-coordinate of v_point

    /*
    Detour triggering regarding NEXT VirtualPoint
     */

    /*
    detour_qs
    q_ij^d: detour trigger
     */
    public GurobiVariable detour_q;

    /*
    Non-overlapping
    0: nonL
    1: nonR
    2: nonT
    3: nonB
     */
    public Map<Obstacle, GurobiVariable[]> non_qs;//binary variables for non-overlapping

    /*
    relD_qs
    0: o_L
    1: o_R
    2: o_T
    3: o_B
     */
    public Map<Obstacle, GurobiVariable[]> relD_qs;//binary variables for (L) (R) (T) (B)

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
    Path Length Computation regarding Next VirtualPoint
     */
    /*
    corner_qs
    0:ll
    1:ur
     */
    public Map<Obstacle, GurobiVariable[]> corner_qs;

    /*
    oCoordinate_iqs
    0: x
    1: y
     */
    public Map<Obstacle, GurobiVariable[]> oCoordinate_iqs;

    /*
    startEndObstacle_qs
    0: vi->
    1: ->vi+1
     */
    public Map<Obstacle, GurobiVariable[]> startEndObstacles_qs;//binaryVar regarding next virtualPoint for starting and end point connection


    /*
    dOut_cqs
     */
    public GurobiVariable dOut_iq;
    /*
    Auxiliary absolute values (Manhattan): aux_dOut_iqs
    For pathLength.stpTo
    0: |x1 - x2|
    1: |y1 - y2|
    2: x1 - x2
    3: y1 - y2
     */
    public Map<Obstacle, GurobiVariable[]> aux_dOut_iqs;

    /*
    dIn_cqs
     */
    public GurobiVariable dIn_iq;
    /*
    Auxiliary absolute values (Manhattan): aux_dIn_iqs
    For pathLength.toEvp
     */
    public Map<Obstacle, GurobiVariable[]> aux_dIn_iqs;

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
     */
    public Map<Obstacle, Map<Obstacle, GurobiVariable>> dOmOn_iq;

    /*
    Auxiliary absolute values (Manhattan): aux_dOmOn_iqs
    For pathLength.mTon
     */
    public Map<Obstacle, Map<Obstacle, GurobiVariable[]>> aux_dOmOn_iqs;

    /*
    linearize Product
    q_m->n * on.relO_q (cnnRules.3)
    q_m->n * om.relO_q (cnnRules.4)
     */
    public Map<Obstacle, Map<Obstacle, GurobiVariable[]>> linAux_OmOn_qs;

    /*
    dist_cqs: vv_dist
     */
    public GurobiVariable dist_iq;

    /*
    Auxiliary absolute values (Manhattan):
    For pathLength vp->vp without detour
     */
    public GurobiVariable[] aux_dist_iqs;

    //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=//
    /*
    Detour triggering regarding Slaves
     */
    //vp->slave

    public GurobiVariable corDist_iq;

    public Map<PseudoBase, GurobiVariable> vsCnn_q;

    public Map<PseudoBase, GurobiVariable> vs_detour_q;

    public Map<PseudoBase, Map<Obstacle, GurobiVariable[]>> vs_relObstaclesD_qs;

    public Map<PseudoBase, Map<Obstacle, GurobiVariable>> vs_relObstacles_q;

    public Map<PseudoBase, Map<Obstacle, GurobiVariable[]>> vs_corner_qs;

    public Map<PseudoBase, Map<Obstacle, GurobiVariable[]>> vs_oCoordinate_iqs;

    public Map<PseudoBase, Map<Obstacle, GurobiVariable[]>> vs_startEndObstacles_qs;

    public Map<PseudoBase, GurobiVariable> vs_dOut_iq;

    public Map<PseudoBase, Map<Obstacle, GurobiVariable[]>> aux_vsdOut_iqs;

    public Map<PseudoBase, GurobiVariable> vs_dIn_iq;

    public Map<PseudoBase, Map<Obstacle, GurobiVariable[]>> aux_vsdIn_iqs;

    public Map<PseudoBase, Map<Obstacle, GurobiVariable[]>> linAux_vsInOut_qs;

    public Map<PseudoBase, Map<Obstacle, Map<Obstacle, GurobiVariable>>> vs_omOnCnn_q;

    public Map<PseudoBase, Map<Obstacle, Map<Obstacle, GurobiVariable>>> vs_dOmOn_iq;

    public Map<PseudoBase, Map<Obstacle, Map<Obstacle, GurobiVariable[]>>> aux_vsdOmOn_iqs;

    public Map<PseudoBase, Map<Obstacle, Map<Obstacle, GurobiVariable[]>>> linAux_vsOmOn_qs;

    public Map<PseudoBase, GurobiVariable> vs_dist_iq;

    public Map<PseudoBase, GurobiVariable[]> aux_vsDist_iqs;


    public RectVirtualPointVar() {
        this.non_qs = new HashMap<>();
        this.relD_qs = new HashMap<>();
        this.relObstaclesD_qs = new HashMap<>();
        this.relObstacles_q = new HashMap<>();

        this.corner_qs = new HashMap<>();
        this.oCoordinate_iqs = new HashMap<>();

        this.startEndObstacles_qs = new HashMap<>();
        this.aux_dOut_iqs = new HashMap<>();
        this.aux_dIn_iqs = new HashMap<>();
        this.linAux_inOut_qs = new HashMap<>();

        this.omOnCnn_q = new HashMap<>();
        this.dOmOn_iq = new HashMap<>();
        this.aux_dOmOn_iqs = new HashMap<>();
        this.linAux_OmOn_qs = new HashMap<>();

        this.vsCnn_q = new HashMap<>();
        this.vs_relObstaclesD_qs = new HashMap<>();
        this.vs_relObstacles_q = new HashMap<>();
        this.vs_detour_q = new HashMap<>();
        this.vs_corner_qs = new HashMap<>();
        this.vs_oCoordinate_iqs = new HashMap<>();
        this.vs_startEndObstacles_qs = new HashMap<>();
        this.vs_dOut_iq = new HashMap<>();
        this.aux_vsdOut_iqs = new HashMap<>();
        this.vs_dIn_iq = new HashMap<>();
        this.aux_vsdIn_iqs = new HashMap<>();
        this.linAux_vsInOut_qs = new HashMap<>();
        this.vs_omOnCnn_q = new HashMap<>();
        this.vs_dOmOn_iq = new HashMap<>();
        this.aux_vsdOmOn_iqs = new HashMap<>();
        this.linAux_vsOmOn_qs = new HashMap<>();
        this.vs_dist_iq = new HashMap<>();
        this.aux_vsDist_iqs = new HashMap<>();

    }
}
