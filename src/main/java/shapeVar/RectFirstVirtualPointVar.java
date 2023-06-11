package shapeVar;

import grb.GurobiVariable;
import shapes.Obstacle;

import java.util.HashMap;
import java.util.Map;

/**
 * @auther lianmeng
 * @create 10.06.23
 */
public class RectFirstVirtualPointVar extends RectVirtualPointVar {

    public Map<Obstacle, GurobiVariable[]> vm_relObstaclesD_qs;

    public Map<Obstacle, GurobiVariable> vm_relObstacles_q;

    public GurobiVariable vm_detour_q;

    /*
    Path Length Computation regarding Next VirtualPoint
     */
    /*
    corner_qs
    0:ll
    1:ur
     */
    public Map<Obstacle, GurobiVariable[]> vm_corner_qs;

    public Map<Obstacle, GurobiVariable[]> vm_oCoordinate_iqs;

    /*
    startEndObstacle_qs
    0: vi->
    1: ->vi+1
     */
    public Map<Obstacle, GurobiVariable[]> vm_startEndObstacles_qs;//binaryVar regarding next virtualPoint for starting and end point connection

    public GurobiVariable vm_dOut_iq;
    /*
    Auxiliary absolute values (Manhattan): aux_dOut_iqs
    For pathLength.stpTo
    0: |x1 - x2|
    1: |y1 - y2|
    2: x1 - x2
    3: y1 - y2
     */
    public Map<Obstacle, GurobiVariable[]> aux_vmdOut_iqs;

    public GurobiVariable vm_dIn_iq;

    public Map<Obstacle, GurobiVariable[]> aux_vmdIn_iqs;

    /*
    linearize Product
    q_->m * om.relO_q (cnnRules.5)
    q_m-> * om.relO_q (cnnRules.6)
     */
    public Map<Obstacle, GurobiVariable[]> linAux_vmInOut_qs;

    public Map<Obstacle, Map<Obstacle, GurobiVariable>> vm_omOnCnn_q;//binaryVar regarding next virtualPoint for relObstacles' connection

    public Map<Obstacle, Map<Obstacle, GurobiVariable>> vm_dOmOn_iq;

    public Map<Obstacle, Map<Obstacle, GurobiVariable[]>> aux_vmdOmOn_iqs;

    /*
    linearize Product
    q_m->n * on.relO_q (cnnRules.3)
    q_m->n * om.relO_q (cnnRules.4)
     */
    public Map<Obstacle, Map<Obstacle, GurobiVariable[]>> linAux_vmOmOn_qs;

    public GurobiVariable vm_dist_iq;

    public GurobiVariable[] aux_vmDist_iqs;

    public RectFirstVirtualPointVar() {
        super();

        this.vm_relObstaclesD_qs = new HashMap<>();
        this.vm_relObstacles_q = new HashMap<>();
        this.vm_corner_qs = new HashMap<>();
        this.vm_oCoordinate_iqs = new HashMap<>();
        this.vm_startEndObstacles_qs = new HashMap<>();
        this.aux_vmdOut_iqs = new HashMap<>();
        this.aux_vmdIn_iqs = new HashMap<>();
        this.linAux_vmInOut_qs = new HashMap<>();
        this.vm_omOnCnn_q = new HashMap<>();
        this.vm_dOmOn_iq = new HashMap<>();
        this.aux_vmdOmOn_iqs = new HashMap<>();
        this.linAux_vmOmOn_qs = new HashMap<>();

    }
}
