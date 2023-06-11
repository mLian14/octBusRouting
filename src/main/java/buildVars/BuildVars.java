package buildVars;

import grb.GurobiExecutor;
import grb.GurobiVariable;
import gurobi.GRB;
import shapes.Obstacle;
import shapes.PseudoBase;

import java.util.ArrayList;

/**
 * @auther lianmeng
 * @create 10.06.23
 */
public class BuildVars {
    public ArrayList<Obstacle> obstacles;
    public ArrayList<PseudoBase> slaves;
    public PseudoBase master;
    public GurobiExecutor executor;
    public int M;

    public BuildVars(ArrayList<Obstacle> obstacles, ArrayList<PseudoBase> slaves, PseudoBase master, GurobiExecutor executor, int m) {
        this.obstacles = obstacles;
        this.slaves = slaves;
        this.master = master;
        this.executor = executor;
        M = m;
    }

    /**
     * auxiliary variables to compute distance regarding Manhattan routing
     * @param varName name
     * @return Array of auxiliary Gurobi variables
     * 0: |x1 - x2|
     * 1: |y1 - y2|
     * 2: x1 - x2
     * 3: y1 - y2
     */
    protected GurobiVariable[] buildAuxManhattanVar(String varName){
        GurobiVariable[] qs = new GurobiVariable[4];
        for (int i = 0; i < 2; ++i) {
            qs[i] = new GurobiVariable(GRB.INTEGER, 0, M, varName + i);
            executor.addVariable(qs[i]);
        }
        for (int i = 2; i < 4; ++i) {
            qs[i] = new GurobiVariable(GRB.INTEGER, -M, M, varName + i);
            executor.addVariable(qs[i]);
        }
        return qs;
    }

    /**
     * build Array of binary variables
     * @param varName name
     * @param varCnt number of variables
     * @return Array of Gurobi binary variables
     */
    protected GurobiVariable[] buildBinaryVar(String varName, int varCnt) {
        GurobiVariable[] qs = new GurobiVariable[varCnt];
        for (int var_cnt = 0; var_cnt < varCnt; ++var_cnt) {
            qs[var_cnt] = new GurobiVariable(GRB.BINARY, 0, 1, varName + var_cnt);
            executor.addVariable(qs[var_cnt]);
        }
        return qs;
    }

    /**
     * build Array of integer variables
     * @param lb lower boundary
     * @param ub upper boundary
     * @param varName name
     * @param varCnt number of variables
     * @return Array of Gurobi integer variables
     */
    protected GurobiVariable[] buildIntVar(int lb, int ub, String varName, int varCnt) {
        GurobiVariable[] qs = new GurobiVariable[varCnt];
        for (int var_cnt = 0; var_cnt < varCnt; ++var_cnt) {
            qs[var_cnt] = new GurobiVariable(GRB.INTEGER, lb, ub, varName + var_cnt);
            executor.addVariable(qs[var_cnt]);
        }
        return qs;
    }
}
