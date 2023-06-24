import gurobi.GRBException;
import parser.OutputDocument;
import processor.Processor;

import java.io.IOException;
import java.time.Duration;
import java.time.LocalDateTime;
import java.time.ZoneId;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;

/**
 * @auther lianmeng
 * @create 26.03.23
 */
public class octALLslavebusrouting {

    public static void main(String[] args) throws IOException, GRBException {
        LocalDateTime start = LocalDateTime.now();
        System.out.println("Program Starts at: " + DateTimeFormatter.ofPattern("yyyy/MM/dd HH:mm:ss.SSS").format(start));
        Processor processor = new Processor();

//        OutputDocument output = processor.processToOutputForFixSlaves("input_octilinear/case1/case1_I2C4");
        ArrayList<Double> totalWireLengths = processor.processSlaveSequence("input_octilinear/case5/case5_I2C2");
//        ArrayList<Double> totalWireLengths = processor.processSlaveSequence("input_octilinear/case1/case1_I2C4");
        for (double length : totalWireLengths){
            System.out.println(length);
        }
//        OutputDocument output = processor.processToOutputForFixSlaves("input_octilinear/case1/case1_I2C0");
//        OutputDocument output = processor.processToOutputForFixSlaves("input_octilinear/case1/case1_I2C6");
//        OutputDocument output = processor.processToOutputForFixSlaves("input_octilinear/case4/case4_I2C0");
//        OutputDocument output = processor.processToOutputForFixSlaves("input_octilinear/case4/case4_I2C3");
//        OutputDocument output = processor.processToOutputForFixSlaves("input_octilinear/case4/case4_I2C4");
//        OutputDocument output = processor.processToOutputForFixSlaves("input_octilinear/case4/case4_I2C6");
//        OutputDocument output = processor.processToOutputForFixSlaves("input_octilinear/case5/case5_I2C0");
//        OutputDocument output = processor.processToOutputForFixSlaves("input_octilinear/case5/case5_I2C1");
//        OutputDocument output = processor.processToOutputForFixSlaves("input_octilinear/case5/case5_I2C2");
//        OutputDocument output = processor.processToOutputForFixSlaves("input_octilinear/case5/case5_I2C4");
//        OutputDocument output = processor.processToOutputForFixSlaves("input_octilinear/case6/case6_I2C0");
//        OutputDocument output = processor.processToOutputForFixSlaves("input_octilinear/case6/case6_I2C1");



        LocalDateTime end = LocalDateTime.now();
        Duration duration = Duration.between(start, end);
        LocalDateTime duration_formated = LocalDateTime.ofInstant(java.time.Instant.ofEpochMilli(duration.toMillis()), ZoneId.of("UTC"));
        System.out.println("Program Ends at: " + DateTimeFormatter.ofPattern("yyyy/MM/dd HH:mm:ss.SSS").format(end));
        System.out.println("Program Run Time is: " + DateTimeFormatter.ofPattern("HH:mm:ss.SSS").format(duration_formated));

    }

}
