package screenMessage;

import grb.GurobiVariable;
import gurobi.GRBException;

/**
 * @auther lianmeng
 * @create 11.06.23
 */
public class RetrieveGurobi {


    protected String convertGrbIntArrayToString(GurobiVariable[] grbIntArray) throws GRBException {
        StringBuilder grbAsString = new StringBuilder("[");
        for (GurobiVariable v : grbIntArray) {
            grbAsString.append(v.getIntResult() + " ");
        }
        grbAsString.delete(grbAsString.length() - 1, grbAsString.length()).append("]");
        return grbAsString.toString();
    }
    protected String convertGrbContArrayToString(GurobiVariable[] grbContArray) throws GRBException {
        StringBuilder grbAsString = new StringBuilder("[");
        for (GurobiVariable v : grbContArray) {
            grbAsString.append(v.getContResult() + " ");
        }
        grbAsString.delete(grbAsString.length() - 1, grbAsString.length()).append("]");
        return grbAsString.toString();
    }

}
