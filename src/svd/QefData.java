package svd;

public class QefData {
    public float ata_00, ata_01, ata_02, ata_11, ata_12, ata_22;
    public float atb_x, atb_y, atb_z;
    public float btb;
    public float massPoint_x, massPoint_y, massPoint_z;
    public int numPoints;

    public QefData()
    {
        clear();
    }

    public QefData(QefData rhs)
    {
        set(rhs);
    }

    public QefData(float ata_00, float ata_01,float ata_02, float ata_11, float ata_12,float ata_22, float atb_x, float atb_y,
                   float atb_z, float btb, float massPoint_x, float massPoint_y, float massPoint_z, int numPoints)
    {

        set(ata_00, ata_01, ata_02, ata_11, ata_12, ata_22, atb_x, atb_y, atb_z, btb, massPoint_x, massPoint_y, massPoint_z, numPoints);
    }

    public void add(QefData rhs)
    {
        ata_00 += rhs.ata_00;
        ata_01 += rhs.ata_01;
        ata_02 += rhs.ata_02;
        ata_11 += rhs.ata_11;
        ata_12 += rhs.ata_12;
        ata_22 += rhs.ata_22;
        atb_x += rhs.atb_x;
        atb_y += rhs.atb_y;
        atb_z += rhs.atb_z;
        btb += rhs.btb;
        massPoint_x += rhs.massPoint_x;
        massPoint_y += rhs.massPoint_y;
        massPoint_z += rhs.massPoint_z;
        numPoints += rhs.numPoints;
    }

    public void clear()
    {
        set(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    }

    public void set(float ata_00, float ata_01, float ata_02, float ata_11, float ata_12, float ata_22, float atb_x, float atb_y,
                    float atb_z, float btb, float massPoint_x, float massPoint_y, float massPoint_z, int numPoints)
    {
        this.ata_00 = ata_00;
        this.ata_01 = ata_01;
        this.ata_02 = ata_02;
        this.ata_11 = ata_11;
        this.ata_12 = ata_12;
        this.ata_22 = ata_22;
        this.atb_x = atb_x;
        this.atb_y = atb_y;
        this.atb_z = atb_z;
        this.btb = btb;
        this.massPoint_x = massPoint_x;
        this.massPoint_y = massPoint_y;
        this.massPoint_z = massPoint_z;
        this.numPoints = numPoints;
    }

    public void set(QefData rhs)
    {
        set(rhs.ata_00, rhs.ata_01, rhs.ata_02, rhs.ata_11, rhs.ata_12,
                rhs.ata_22, rhs.atb_x, rhs.atb_y, rhs.atb_z, rhs.btb,
                rhs.massPoint_x, rhs.massPoint_y, rhs.massPoint_z,
                rhs.numPoints);
    }
}
