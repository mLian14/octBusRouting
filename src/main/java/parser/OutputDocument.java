package parser;

import shapes.Obstacle;
import shapes.PseudoBase;

import java.util.ArrayList;

/**
 * @auther lianmeng
 * @create 26.03.23
 */
public class OutputDocument {


    private final String name;
    private final PseudoBase master;
    private final ArrayList<Obstacle> obstacles;

    private ArrayList<PseudoBase> virtualPoints;
    private ArrayList<PseudoBase> slaves;

    public OutputDocument(String name, PseudoBase master, ArrayList<Obstacle> obstacles) {
        this.name = name;
        this.master = master;
        this.obstacles = obstacles;
        this.virtualPoints = new ArrayList<>();
        this.slaves = new ArrayList<>();
    }

    public ArrayList<PseudoBase> getVirtualPoints() {
        return virtualPoints;
    }

    public void setVirtualPoints(ArrayList<PseudoBase> virtualPoints) {
        this.virtualPoints = virtualPoints;
    }

    public ArrayList<PseudoBase> getSlaves() {
        return slaves;
    }

    public void setSlaves(ArrayList<PseudoBase> slaves) {
        this.slaves = slaves;
    }

    public ArrayList<Obstacle> getObstacles() {
        return obstacles;
    }

    public String getName() {
        return name;
    }

    public PseudoBase getMaster() {
        return master;
    }
}
