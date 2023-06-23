package shapes;

import java.util.ArrayList;
import java.util.Map;

/**
 * @auther lianmeng
 * @create 11.06.23
 */
public class SteinerPoint extends PseudoBase{

    public SteinerPoint(int x, int y, String name) {
        super(x, y);
        this.setName(name);
    }

    Map<PseudoBase, ArrayList<Obstacle>> basePaths;
    Map<SteinerPoint, ArrayList<Obstacle>> steinerPaths;



}
