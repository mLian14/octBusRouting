package drawConnections;

import parser.OutputDocument;
import shapes.PseudoBase;

import java.util.ArrayList;

/**
 * @auther lianmeng
 * @create 11.06.23
 */
public class DrawConnections {
    public OutputDocument output;
    public final String path;

    public DrawConnections(OutputDocument output, String path) {
        this.output = output;
        this.path = path;
    }
}
