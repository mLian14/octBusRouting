package shapeVar;

import grb.GurobiVariable;
import shapes.Obstacle;
import shapes.PseudoBase;

import java.util.HashMap;
import java.util.Map;

/**
 * @auther lianmeng
 * @create 08.06.23
 */
public class OctFirstVirtualPointFixSlaveVar extends OctVirtualPointFixSlaveVar {

    /*
    vm:relObstacles_qs
    0: ul->lr
    1: lr->ul
    2: ur->ll
    3: ll->ur
     */
    public Map<Obstacle, GurobiVariable[]> vm_relObstacles_qs;//binary variables for (ul->lr.1) -- (ll->ur.1)

    /*
    vm:relObstaclesD_qs
    0: l->r
    1: r->l
    2: t->b
    3: b->t
     */
    public Map<Obstacle, GurobiVariable[]> vm_relObstaclesD_qs;

    /*
    indicate relevant obstacles
     */
    public Map<Obstacle, GurobiVariable> vm_relObstacles_q;


    /*
    VM:detour_qs
    q_ij^d: detour trigger
     */
    public GurobiVariable vm_detour_q;//binary variables regarding next virtualPoint for (ul->lr.2) -- (ll->ur.2)

    /*
    vm_enterCorner_qs, vm_leaveCorner_qs
    0: ll
    1: ur
    2: ul
    3: lr
     */
    public Map<Obstacle, GurobiVariable[]> vm_enterCorner_qs;
    public Map<Obstacle, GurobiVariable[]> vm_leaveCorner_qs;



    /*
    vm:omOnCnn_q
    q_i_vj^m->n
     */
    public Map<Obstacle, Map<Obstacle, GurobiVariable>> vm_omOnCnn_q;//binaryVar regarding next virtualPoint for relObstacles' connection

    /*
    vm:inOutCnn_qs
    0: vi -> obstacle
    1: obstacle -> vj
     */
    public Map<Obstacle, GurobiVariable[]> vm_startEndObstacles_qs;//binaryVar regarding next virtualPoint for starting and end point connection

    /*
    vm_enterCoordinate_iqs, vm_leaveCoordinate_iqs
    0: x_m
    1: y_m
     */
    public Map<Obstacle, GurobiVariable[]> vm_enterCoordinate_iqs;
    public Map<Obstacle, GurobiVariable[]> vm_leaveCoordinate_iqs;
    /*
    Auxiliary absolute values (Manhattan): aux_vmdOm_iqs
    Manhattan distance between enter and leave corners
     */
    public Map<Obstacle, GurobiVariable[]> aux_vmdOm_iqs;

    /*
    vm: dOut_cqs
    0: dv0->: min
    1: dv0->: xy
     */
    public GurobiVariable[] vm_dOut_cqs;
    /*
    Auxiliary absolute values: aux_vmdOut_iqs
    For pathLength.svpTo
    0: |vi.x - x_m|
    1: |vi.y - y_m|
    2: min(|vi.x - x_m|, |vi.y - y_m|)
    3: ||vi.x - x_m| - |vi.y - y_m||
    4: vi.x - x_m
    5: vi.y - y_m
    6: |vi.x - x_m| - |vi.y - y_m|
     */
    public Map<Obstacle, GurobiVariable[]> aux_vmdOut_iqs;
    public Map<Obstacle, GurobiVariable> auxQ_vmdOut;

    /*
    vm: dIn_cqs
    0: d->ms: min
    1: d->ms: xy
     */
    public GurobiVariable[] vm_dIn_cqs;
    /*
    Auxiliary absolute values: aux_vmdIn_iqs
    For pathLength.toMaster
     */
    public Map<Obstacle, GurobiVariable[]> aux_vmdIn_iqs;
    public Map<Obstacle, GurobiVariable> auxQ_vmdIn;
    /*
    linearize Product
    q_->m * om.relO_q (cnnRules.5)
    q_m-> * om.relO_q (cnnRules.6)
     */
    public Map<Obstacle, GurobiVariable[]> linAux_vmInOut_qs;



    /*
    vm:dOmOn_cqs
    0: d_m->n: min
    1: d_m->n: xy
     */
    public Map<Obstacle, Map<Obstacle, GurobiVariable[]>> vm_dOmOn_cqs;//intVar: path length between o_m and o_n


    /*
    Auxiliary absolute values: aux_vmdOmOn_iqs
    For pathLength.mTon
     */
    public Map<Obstacle, Map<Obstacle, GurobiVariable[]>> aux_vmdOmOn_iqs;
    public Map<Obstacle, Map<Obstacle, GurobiVariable>> auxQ_vmdOmOn;

    /*
    linearize Product
    q_m->n * on.relO_q (cnnRules.3)
    q_m->n * om.relO_q (cnnRules.4)
     */
    public Map<Obstacle, Map<Obstacle, GurobiVariable[]>> linAux_vmOmOn_qs;

    /*
    vm_dist_cqs:
    0: vm_dist: min
    1: vm_dist: xy
     */
    public GurobiVariable[] vm_dist_cqs;

    /*
    Auxiliary absolute values:
    For pathLength v0->master without detour
     */
    public GurobiVariable[] aux_vm_dist_iqs;
    public GurobiVariable auxQ_vm_dist;


    public OctFirstVirtualPointFixSlaveVar(PseudoBase slave) {
        super(slave);
        this.vm_relObstacles_qs = new HashMap<>();
        this.vm_relObstaclesD_qs = new HashMap<>();
        this.vm_relObstacles_q = new HashMap<>();

        this.vm_enterCorner_qs = new HashMap<>();
        this.vm_leaveCorner_qs = new HashMap<>();
        this.aux_vmdOm_iqs = new HashMap<>();
        this.vm_omOnCnn_q = new HashMap<>();
        this.vm_startEndObstacles_qs = new HashMap<>();
        this.vm_enterCoordinate_iqs = new HashMap<>();
        this.vm_leaveCoordinate_iqs = new HashMap<>();

        this.aux_vmdOut_iqs = new HashMap<>();
        this.auxQ_vmdOut = new HashMap<>();
        this.aux_vmdIn_iqs = new HashMap<>();
        this.auxQ_vmdIn = new HashMap<>();
        this.aux_vmdOmOn_iqs = new HashMap<>();
        this.auxQ_vmdOmOn = new HashMap<>();



        this.vm_dOmOn_cqs = new HashMap<>();
        this.linAux_vmInOut_qs = new HashMap<>();
        this.linAux_vmOmOn_qs = new HashMap<>();



    }
}
