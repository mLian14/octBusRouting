package processor;

import buildCons.BuildConsForFixSlaves;
import buildCons.BuildConsForRectRouting;
import buildVars.BuildVarsForFixSlaves;
import buildVars.BuildVarsForRectRouting;
import grb.GurobiExecutor;
import grb.GurobiObjConstraint;
import grb.GurobiVariable;
import gurobi.GRB;
import gurobi.GRBConstr;
import gurobi.GRBException;
import parser.Document;
import parser.DocumentParser;
import parser.OutputDocument;
import screenMessage.ScreenMessageFixSlaves;
import screenMessage.ScreenMessageRect;
import shapeVar.OctVirtualPointFixSlaveVar;
import shapeVar.RectVirtualPointVar;
import shapes.Obstacle;
import shapes.PseudoBase;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

/**
 * @auther lianmeng
 * @create 08.06.23
 */
public class Processor {

    private GurobiExecutor executor;
    private final DocumentParser parser;
    private final static int M = 99999;
    private OutputDocument output;


    public Processor() {
        this.parser = new DocumentParser();
    }

    public ArrayList<Double> processSlaveSequence(String path, int desiredCount) throws GRBException{
        parser.parseInputToDocument(path);
        Document input = parser.getParseDoc();
        String[] names = path.split("/");
        input.setName(names[names.length - 1]);

        ArrayList<Double> totalWireLengths = new ArrayList<>();
        List<ArrayList<PseudoBase>> slavePermutations = PermutationGenerator.generateRandomPermutations(input.getSlaves(), desiredCount);

        for (int i = 0; i < slavePermutations.size(); i ++){
            ArrayList<PseudoBase> newSlaves = slavePermutations.get(i);
            for (PseudoBase sv : newSlaves){
                System.out.print(sv.getName() + "-");
            }
            System.out.println();
            OutputDocument output = processToOutputForFixSlaves(input.getName(), input.getMaster(), newSlaves, input.getObstacles(), input.getBusC(), input.getSlaveC());
            totalWireLengths.add(output.getTotalWireLength());
        }
        return totalWireLengths;
    }

    public OutputDocument processToOutputForFixSlaves(String path) throws GRBException {

        parser.parseInputToDocument(path);
        Document input = parser.getParseDoc();
        String[] names = path.split("/");
        input.setName(names[names.length - 1]);
        return processToOutputForFixSlaves(input.getName(), input.getMaster(), input.getSlaves(), input.getObstacles(), input.getBusC(), input.getSlaveC());
    }
    public OutputDocument processToOutputForFixSlaves(String caseName, PseudoBase master, ArrayList<PseudoBase> slaves, ArrayList<Obstacle> obstacles, double busC, double slaveC) throws GRBException {
        OutputDocument output = new OutputDocument(caseName, master, obstacles);
        /*
        Determine the pseudo Variables of Master and slaves
         */
        pseudoBaseVariablesDetermination(master, slaves, obstacles);

        /*
        Gurobi
        Start
        VVVV
         */
        executor = new GurobiExecutor("octilinearBusRouting");
        executor.setMIPGap(0.02);
        executor.setMIPGapAbs(0.02);
//        executor.setTimeLimit(200);
        executor.setTimeLimit(36000);

        //build Gurobi Variables

        GurobiVariable busMin, busXY, branchMin, branchXY;
        busMin = new GurobiVariable(GRB.INTEGER, 0, M, "busLengthMin");
        executor.addVariable(busMin);
        busXY = new GurobiVariable(GRB.INTEGER, 0, M, "busLengthXY");
        executor.addVariable(busXY);
        branchMin = new GurobiVariable(GRB.INTEGER, 0, M, "branchLengthMin");
        executor.addVariable(branchMin);
        branchXY = new GurobiVariable(GRB.INTEGER, 0, M, "branchLengthXY");
        executor.addVariable(branchXY);

        ArrayList<OctVirtualPointFixSlaveVar> vps = new ArrayList<>();
        BuildVarsForFixSlaves buildVarsForFixSlaves = new BuildVarsForFixSlaves(obstacles, vps, slaves, master, executor, M);
        buildVarsForFixSlaves.buildVariables();
        executor.updateModelWithVars();
        System.out.println("#Variables = " + executor.getVariables().size());

        int minDist = 1;
        BuildConsForFixSlaves buildCons = new BuildConsForFixSlaves(obstacles, vps, slaves, master, busMin, busXY, branchMin, branchXY, executor, M, minDist);
        buildCons.buildConstraints();
        executor.updateModelWithCons();
        System.out.println("#Constraints = " + executor.getConstraints().size());

        //Objective Function
        GurobiObjConstraint objConstraint = new GurobiObjConstraint();
        objConstraint.addToLHS(busMin, Math.sqrt(2) * busC);
        objConstraint.addToLHS(busXY, busC);
        objConstraint.addToLHS(branchMin, Math.sqrt(2) * slaveC);
        objConstraint.addToLHS(branchXY, slaveC);

        objConstraint.setGoal(GRB.MINIMIZE);
        executor.setObjConstraint(objConstraint);

        executor.startOptimization();
        this.executor.getModel().write("debug.lp");
        int status = executor.getModel().get(GRB.IntAttr.Status);

        gurobiStatus(status);

        ScreenMessageFixSlaves screenMessage = new ScreenMessageFixSlaves(output, obstacles, vps, master, busMin, busXY, branchMin, branchXY);
        screenMessage.showResult();


        // Dispose of model and environment
        executor.getModel().dispose();
        executor.getEnv().dispose();
        return output;
    }



    public OutputDocument processToOutputForRectRouting(String path) throws GRBException {

        parser.parseInputToDocument(path);
        Document input = parser.getParseDoc();
        String[] names = path.split("/");
        input.setName(names[names.length - 1]);
        return processToOutputForRectRouting(input.getName(), input.getMaster(), input.getSlaves(), input.getObstacles(), input.getBusC(), input.getSlaveC());
    }

    public OutputDocument processToOutputForRectRouting(String caseName, PseudoBase master, ArrayList<PseudoBase> slaves, ArrayList<Obstacle> obstacles, double busC, double slaveC) throws GRBException{

        OutputDocument output = new OutputDocument(caseName, master, obstacles);
        /*
        Determine the pseudo Variables of Master and slaves
         */
        pseudoBaseVariablesDetermination(master, slaves, obstacles);

        /*
        Gurobi
        Start
        VVVV
         */
        executor = new GurobiExecutor("octilinearBusRouting");
        executor.setMIPGap(0);
        executor.setMIPGapAbs(0);
//        executor.setTimeLimit(200);
        executor.setTimeLimit(36000);

        //build Gurobi Variables

        GurobiVariable busLength,branchLength;
        busLength = new GurobiVariable(GRB.INTEGER, 0, M, "busLength");
        executor.addVariable(busLength);
        branchLength = new GurobiVariable(GRB.INTEGER, 0, M, "branchLength");
        executor.addVariable(branchLength);


        ArrayList<RectVirtualPointVar> vps = new ArrayList<>();
        BuildVarsForRectRouting buildVarsForRectRouting = new BuildVarsForRectRouting(vps, obstacles, slaves, master, executor, M);
        buildVarsForRectRouting.buildVars();
        executor.updateModelWithVars();
        System.out.println("#Variables = " + executor.getVariables().size());

        BuildConsForRectRouting buildConsForRectRouting = new BuildConsForRectRouting(vps, obstacles, slaves, master, busLength, branchLength, executor, M, 1);
        buildConsForRectRouting.buildCons();
        executor.updateModelWithCons();
        System.out.println("#Constraints= " + executor.getConstraints().size());

        //Objective Function
        GurobiObjConstraint objConstraint = new GurobiObjConstraint();
        objConstraint.addToLHS(busLength, busC);
        objConstraint.addToLHS(branchLength, slaveC);

        objConstraint.setGoal(GRB.MINIMIZE);
        executor.setObjConstraint(objConstraint);

        executor.startOptimization();
        this.executor.getModel().write("debug.lp");
        int status = executor.getModel().get(GRB.IntAttr.Status);

        gurobiStatus(status);

        ScreenMessageRect screenMessage = new ScreenMessageRect(output, obstacles, vps, slaves, master, busLength, branchLength);
        screenMessage.showResult();


        return output;
    }


    /**
     * Octilinear Routing: Detect the direction and relation between known points, i.e., Master and Slaves, and obstacles.
     *
     * @param master    master
     * @param slaves    ArrayList of slaves
     * @param obstacles ArrayList of obstacles
     */
    public void pseudoBaseVariablesDetermination(PseudoBase master, ArrayList<PseudoBase> slaves, ArrayList<Obstacle> obstacles) {

        for (Obstacle o : obstacles) {

            basicBinaryVariables(master, o);
            for (PseudoBase slave : slaves) {
                basicBinaryVariables(slave, o);
            }


            /*
            oo_dir
             */
            for (Obstacle other_o : obstacles) {
                if (!other_o.getName().equals(o.getName())) {

                    //OtL:
                    if (o.atL(other_o)) {
                        o.addTotLObstacles(other_o);
                    }

                    //OtR:
                    if (o.atR(other_o)) {
                        o.addTotRObstacles(other_o);
                    }

                    //ObL:
                    if (o.abL(other_o)) {
                        o.addTobLObstacles(other_o);
                    }

                    //ObR:
                    if (o.abR(other_o)) {
                        o.addTobRObstacles(other_o);
                    }

                    //OdL:
                    if (o.odL(other_o)) {
                        o.addTodLObstacles(other_o);
                    }
                    //OdR:
                    if (o.odR(other_o)) {
                        o.addTodRObstacles(other_o);
                    }
                    //OdT:
                    if (o.odT(other_o)) {
                        o.addTodTObstacles(other_o);
                    }
                    //OdB:
                    if (o.odB(other_o)) {
                        o.addTodBObstacles(other_o);
                    }


                } else continue;
            }
        }


    }
    /**
     * Octilinear Routing: Determine dir_q, rel_q, relD_q
     *
     * @param base given point
     * @param o    given obstacle
     */
    private void basicBinaryVariables(PseudoBase base, Obstacle o) {

        /*
        oDir_q: UL, UR, LR, LL, L, R, T, B
         */
        int[] odir_q = new int[8];
        //UL
        if (base.getY() < base.getX() + o.getMaxY() - o.getMinX()) {
            odir_q[0] = 0;
        } else
            odir_q[0] = 1;
        //UR
        if (base.getY() <= -base.getX() + o.getMaxY() + o.getMaxX()) {
            odir_q[1] = 0;
        } else
            odir_q[1] = 1;
        //LR
        if (base.getY() <= base.getX() + o.getMinY() - o.getMaxX()) {
            odir_q[2] = 1;
        } else
            odir_q[2] = 0;
        //LL
        if (base.getY() < -base.getX() + o.getMinY() + o.getMinX()) {
            odir_q[3] = 1;
        } else
            odir_q[3] = 0;
        //L
        if (base.getX() < o.getMinX()) {
            odir_q[4] = 0;
        } else
            odir_q[4] = 1;
        //R
        if (base.getX() > o.getMaxX()) {
            odir_q[5] = 0;
        } else
            odir_q[5] = 1;
        //T
        if (base.getY() > o.getMaxY()) {
            odir_q[6] = 0;
        } else
            odir_q[6] = 1;
        //B
        if (base.getY() < o.getMinY()) {
            odir_q[7] = 0;
        } else
            odir_q[7] = 1;
        base.addToPseudo_oDir_qs(o, odir_q);

        /*
        oRel_q:
         */
        int[] orel_q = new int[8];
        //UpperLeft
        if (odir_q[3] + odir_q[1] + odir_q[4] * odir_q[6] == 0) {
            orel_q[0] = 1;
        } else orel_q[0] = 0;
        //UpperRight
        if (odir_q[0] + odir_q[2] + odir_q[5] * odir_q[6] == 0) {
            orel_q[1] = 1;
        } else orel_q[1] = 0;
        //LowerLeft
        if (odir_q[0] + odir_q[2] + odir_q[4] * odir_q[7] == 0) {
            orel_q[2] = 1;
        } else orel_q[2] = 0;
        //LowerRight
        if (odir_q[1] + odir_q[3] + odir_q[5] * odir_q[7] == 0) {
            orel_q[3] = 1;
        } else orel_q[3] = 0;

        //d_Left
        if (odir_q[5] + odir_q[6] + odir_q[7] == 3) {
            orel_q[4] = 1;
        } else orel_q[4] = 0;
        //d_Right
        if (odir_q[4] + odir_q[6] + odir_q[7] == 3) {
            orel_q[5] = 1;
        } else orel_q[5] = 0;
        //d_Top
        if (odir_q[4] + odir_q[5] + odir_q[7] == 3) {
            orel_q[6] = 1;
        } else orel_q[6] = 0;
        //d_Bottom
        if (odir_q[4] + odir_q[5] + odir_q[6] == 3) {
            orel_q[7] = 1;
        } else orel_q[7] = 0;


        base.addToPseudo_oRel_qs(o, orel_q);

    }


    private void gurobiStatus(int status) throws GRBException {
        if (status == GRB.Status.UNBOUNDED) {
            System.out.println("The model cannot be solved " + "because it is unbounded");
        } else if (status == GRB.Status.OPTIMAL) {
            System.out.println("The optimal objective is " + executor.getModel().get(GRB.DoubleAttr.ObjVal));
        } else if (status != GRB.Status.INF_OR_UNBD && status != GRB.Status.INFEASIBLE) {
            System.out.println("Optimization was stopped with status " + status);
        } else if (executor.getModel().get(GRB.IntAttr.Status) == GRB.INFEASIBLE) {
            executor.whenInfeasibleWriteOutputTo("inf.ilp");
            System.out.println("HH:IIS");


            // Do IIS
            System.out.println("The model is infeasible; computing IIS");
            LinkedList<String> removed = new LinkedList<String>();

            // Loop until we reduce to a model that can be solved
            while (true) {
                executor.getModel().computeIIS();
                System.out.println("\nThe following constraint cannot be satisfied:");
                for (GRBConstr c : executor.getModel().getConstrs()) {
                    if (c.get(GRB.IntAttr.IISConstr) == 1) {
                        System.out.println(c.get(GRB.StringAttr.ConstrName));
                        // Remove a single constraint from the model
                        removed.add(c.get(GRB.StringAttr.ConstrName));
                        executor.getModel().remove(c);
                        break;
                    }
                }

                System.out.println();
                executor.getModel().optimize();
                status = executor.getModel().get(GRB.IntAttr.Status);

                if (status == GRB.Status.UNBOUNDED) {
                    System.out.println("The model cannot be solved because it is unbounded");
                    break;
                }
                if (status == GRB.Status.OPTIMAL) {
                    break;
                }
                if (status != GRB.Status.INF_OR_UNBD &&
                        status != GRB.Status.INFEASIBLE) {
                    System.out.println("Optimization was stopped with status " +
                            status);
                    break;
                }


            }
            System.out.println("\nThe following constraints were removed to get a feasible LP:");
            for (String s : removed) {
                System.out.print(s + " ");
            }
            System.out.println();
        }
    }



}
