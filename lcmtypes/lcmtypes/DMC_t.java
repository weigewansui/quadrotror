/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package lcmtypes;
 
import java.io.*;
import java.util.*;
import lcm.lcm.*;
 
public final class DMC_t implements lcm.lcm.LCMEncodable
{
    public long timestamp;
    public short rpm[];
 
    public DMC_t()
    {
        rpm = new short[4];
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0xd7ed511cb14f2cf0L;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class<?>>());
    }
 
    public static long _hashRecursive(ArrayList<Class<?>> classes)
    {
        if (classes.contains(lcmtypes.DMC_t.class))
            return 0L;
 
        classes.add(lcmtypes.DMC_t.class);
        long hash = LCM_FINGERPRINT_BASE
            ;
        classes.remove(classes.size() - 1);
        return (hash<<1) + ((hash>>63)&1);
    }
 
    public void encode(DataOutput outs) throws IOException
    {
        outs.writeLong(LCM_FINGERPRINT);
        _encodeRecursive(outs);
    }
 
    public void _encodeRecursive(DataOutput outs) throws IOException
    {
        outs.writeLong(this.timestamp); 
 
        for (int a = 0; a < 4; a++) {
            outs.writeShort(this.rpm[a]); 
        }
 
    }
 
    public DMC_t(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public DMC_t(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static lcmtypes.DMC_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        lcmtypes.DMC_t o = new lcmtypes.DMC_t();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.timestamp = ins.readLong();
 
        this.rpm = new short[(int) 4];
        for (int a = 0; a < 4; a++) {
            this.rpm[a] = ins.readShort();
        }
 
    }
 
    public lcmtypes.DMC_t copy()
    {
        lcmtypes.DMC_t outobj = new lcmtypes.DMC_t();
        outobj.timestamp = this.timestamp;
 
        outobj.rpm = new short[(int) 4];
        System.arraycopy(this.rpm, 0, outobj.rpm, 0, 4); 
        return outobj;
    }
 
}

