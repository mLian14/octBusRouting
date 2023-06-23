package drawConnections;

import com.itextpdf.kernel.colors.Color;
import com.itextpdf.kernel.colors.DeviceRgb;
import com.itextpdf.kernel.geom.PageSize;
import com.itextpdf.kernel.geom.Rectangle;
import com.itextpdf.kernel.pdf.PdfDocument;
import com.itextpdf.kernel.pdf.PdfWriter;
import com.itextpdf.kernel.pdf.canvas.PdfCanvas;
import com.itextpdf.layout.Document;
import com.itextpdf.layout.element.Paragraph;
import parser.OutputDocument;
import shapes.Obstacle;
import shapes.PseudoBase;

import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Collections;

import static com.itextpdf.kernel.colors.Color.convertRgbToCmyk;

/**
 * @auther lianmeng
 * @create 11.06.23
 */
public class DrawRectConnections extends DrawConnections{

    public DrawRectConnections(OutputDocument output, String path) {
        super(output, path);
    }

    public void outputToPdf() throws FileNotFoundException {


        ArrayList<PseudoBase> virtualPoints = output.getVirtualPoints();
        ArrayList<PseudoBase> slaves = output.getSlaves();
        ArrayList<Obstacle> obstacles = output.getObstacles();
        PseudoBase master = output.getMaster();
        /*
        Process the PDFsize
         */
        ArrayList<Integer> xs = new ArrayList<>();
        ArrayList<Integer> ys = new ArrayList<>();
        xs.add(master.getX());
        ys.add(master.getY());
        for (PseudoBase vp : virtualPoints){
            xs.add(vp.getX());
            ys.add(vp.getY());
        }
        for (PseudoBase sv : slaves){
            xs.add(sv.getX());
            ys.add(sv.getY());
        }
        for (Obstacle o : obstacles){
            xs.add(o.getMinX());
            xs.add(o.getMaxX());
            ys.add(o.getMinY());
            ys.add(o.getMaxY());
        }
        int lb_x = Collections.min(xs);
        int ub_x = Collections.max(xs);
        int lb_y = Collections.min(ys);
        int ub_y = Collections.max(ys);
        lb_x -= 100;
        ub_x += 100;
        lb_y -= 100;
        ub_y += 100;
        int width = ub_x - lb_x;
        int height = ub_y - lb_y;
        int sum_wh = width + height;

        double master_r = sum_wh/200;
        double vp_r = sum_wh/300;
        double sl_r = sum_wh/350;
        float lineWidth = sum_wh/1000;



        //String pdfPath = this.path + "_" + outputDoc.getI2Cname() + ".pdf";
        String pdfPath = this.path + ".pdf";
        Rectangle rectangle = new Rectangle(lb_x, lb_y, width, height);
        PdfDocument pdfDoc = new PdfDocument(new PdfWriter(pdfPath));
        Document document = new Document(pdfDoc, new PageSize(rectangle));


        PdfCanvas canvas = new PdfCanvas(pdfDoc.addNewPage());
        canvas.setLineWidth(lineWidth);

        /*
        draw Keepouts
         */
        float p_bias = 2;
        for (Obstacle o : obstacles){
            Rectangle rect = new Rectangle(o.getMinX(), o.getMinY(), o.getMaxX() - o.getMinX(), o.getMaxY() - o.getMinY());
            Color LightBlue = convertRgbToCmyk(new DeviceRgb(173,216,230));
            canvas.setColor(LightBlue, true)
                    .rectangle(rect)
                    .fill()
                    .stroke();
            Color BLACK = convertRgbToCmyk(new DeviceRgb(0,0,0));
            canvas.setColor(BLACK, false)
                    .rectangle(rect)
                    .stroke();
            Paragraph pO = new Paragraph("[" + o.getMinX() + ", " + o.getMaxX() + "]*[" + o.getMinY() + ", " + o.getMaxY() + "]").setFontSize(20).setFontColor(convertRgbToCmyk(new DeviceRgb(0,0,0)));
            pO.setFixedPosition(o.getMinX() + p_bias, (float) ((o.getMinY() + o.getMaxY())*0.5), 2000);
            document.add(pO);
        }
        /*
        draw Master
         */

        Color PINK = convertRgbToCmyk(new DeviceRgb(255,182,193));
        canvas.setColor(PINK, true)
                .circle(master.getX(), master.getY(), master_r)
                .fill()
                .stroke();
        Paragraph pMaster = new Paragraph(master.getName() + " (" + master.getX() + ", " + master.getY() + ")").setFontSize(20).setFontColor(convertRgbToCmyk(new DeviceRgb(0,0,0)));
        pMaster.setFixedPosition(master.getX() , master.getY() + p_bias, 200);
        document.add(pMaster);

        int vp_cnt = 0;


        /*
        draw Slaves
         */

        for (PseudoBase sv : slaves){
            Color RED = convertRgbToCmyk(new DeviceRgb(255,0,0));
            canvas.setColor(RED, true)
                    .circle(sv.getX(), sv.getY(), sl_r)
                    .fill()
                    .stroke();

            //Paragraph pSV = new Paragraph(sv.getName() +" (" + sv.getX() + ", " + sv.getY() + ")").setFontSize(20).setFontColor(convertRgbToCmyk(new DeviceRgb(0,0,0)));
            Paragraph pSV = new Paragraph(sv.getName()).setFontSize(20).setFontColor(convertRgbToCmyk(new DeviceRgb(0,0,0)));
            pSV.setFixedPosition(sv.getX() , sv.getY() - p_bias, 2000);
            document.add(pSV);
        }



        pdfDoc.close();




    }
}
