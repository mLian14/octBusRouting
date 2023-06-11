package buildCons;

import grb.GurobiConstraint;
import grb.GurobiExecutor;
import grb.GurobiVariable;
import gurobi.GRBException;
import shapes.Obstacle;
import shapes.PseudoBase;

import java.util.ArrayList;

/**
 * @auther lianmeng
 * @create 10.06.23
 */
public class BuildCons {
    public ArrayList<Obstacle> obstacles;
    public ArrayList<PseudoBase> slaves;
    public PseudoBase master;

    public GurobiExecutor executor;
    public int M;
    public int minDist;

    public BuildCons(ArrayList<Obstacle> obstacles, ArrayList<PseudoBase> slaves, PseudoBase master, GurobiExecutor executor, int m, int minDist) {
        this.obstacles = obstacles;
        this.slaves = slaves;
        this.master = master;
        this.executor = executor;
        M = m;
        this.minDist = minDist;
    }

    /**
     * Give the distance regarding Manhattan geometry between enter and leave corners
     * @param nickName
     * @param x1
     * @param y1
     * @param x2
     * @param y2
     * @param aux_dist_iqs store the result
     * @throws GRBException Gurobi
     */
    protected void auxiliaryManhattanDistCons(String nickName, GurobiVariable x1, GurobiVariable y1, GurobiVariable x2, GurobiVariable y2, GurobiVariable[] aux_dist_iqs) throws GRBException{
        GurobiConstraint c;

        c = new GurobiConstraint();
        c.setName(nickName + "_aux_x");
        c.addToLHS(x1, 1.0);
        c.addToLHS(x2, -1.0);
        c.setSense('=');
        c.addToRHS(aux_dist_iqs[2], 1.0);
        executor.addConstraint(c);
        executor.addGenConstraintAbs(aux_dist_iqs[0], aux_dist_iqs[2], nickName + "absX");
        c = new GurobiConstraint();
        c.setName(nickName + "_aux_Y");
        c.addToLHS(y1, 1.0);
        c.addToLHS(y2, -1.0);
        c.setSense('=');
        c.addToRHS(aux_dist_iqs[3], 1.0);
        executor.addConstraint(c);
        executor.addGenConstraintAbs(aux_dist_iqs[1], aux_dist_iqs[3], nickName + "absY");
    }

    /**
     * Give the distance regarding Manhattan geometry between enter and leave corners
     * @param nickName name
     * @param cor1 coordinate1
     * @param cor2 coordinate2
     * @param aux_dist_iqs store the result
     * @throws GRBException Gurobi
     */
    protected void auxiliaryManhattanDistCons(String nickName, GurobiVariable[] cor1, GurobiVariable[] cor2, GurobiVariable[] aux_dist_iqs) throws GRBException{
        GurobiVariable x1 = cor1[0];
        GurobiVariable y1 = cor1[1];
        GurobiVariable x2 = cor2[0];
        GurobiVariable y2 = cor2[1];

        auxiliaryManhattanDistCons(nickName, x1, y1, x2, y2, aux_dist_iqs);
    }

    /**
     * If all Var in ArrayList is 0, the constraints implies targetVar to be 1
     * @param nickname
     * @param targetVar
     * @param variables
     */
    protected void allZeroBVars(String nickname, GurobiVariable targetVar, ArrayList<GurobiVariable> variables) {
        GurobiConstraint c;
        c = new GurobiConstraint();
        c.setName(nickname + "_leq");
        for (GurobiVariable var : variables) {
            c.addToLHS(var, 1.0);
        }
        c.setSense('<');
        c.addToRHS(targetVar, -variables.size());
        c.setRHSConstant(variables.size());
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName(nickname + "_geq");
        for (GurobiVariable var : variables) {
            c.addToLHS(var, 1.0);
        }
        c.setSense('>');
        c.addToRHS(targetVar, -1.0);
        c.setRHSConstant(1.0);
        executor.addConstraint(c);
    }

    /**
     * If all Var in ArrayList is 1, the constraints implies targetVar to be 1
     * @param nickname
     * @param targetVar
     * @param variables
     */
    protected void allOneBVars(String nickname, GurobiVariable targetVar, ArrayList<GurobiVariable> variables) {
        GurobiConstraint c;

        c = new GurobiConstraint();
        c.setName(nickname + "_geq");
        for (GurobiVariable var : variables) {
            c.addToLHS(var, 1.0);
        }
        c.setSense('>');
        c.addToRHS(targetVar, variables.size());
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.setName(nickname + "_leq");
        for (GurobiVariable var : variables) {
            c.addToLHS(var, 1.0);
        }
        c.setSense('<');
        c.addToRHS(targetVar, 1.0);
        c.setRHSConstant(variables.size() - 1.0);
        executor.addConstraint(c);
    }

    /**
     * Linearize the product of two binary variables
     * @param product
     * @param var1
     * @param var2
     */
    protected void linearizeProduct2BVars(GurobiVariable product, GurobiVariable var1, GurobiVariable var2) {
        GurobiConstraint c;
        c = new GurobiConstraint();
        c.addToLHS(product, 1.0);
        c.setSense('<');
        c.addToRHS(var1, 1.0);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.addToLHS(product, 1.0);
        c.setSense('<');
        c.addToRHS(var2, 1.0);
        executor.addConstraint(c);
        c = new GurobiConstraint();
        c.addToLHS(product, 1.0);
        c.setSense('>');
        c.addToRHS(var1, 1.0);
        c.addToRHS(var2, 1.0);
        c.setRHSConstant(-1.0);
        executor.addConstraint(c);
    }

}
