/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package lcmtypes;
 
import java.io.*;
import java.util.*;
import lcm.lcm.*;
 
public final class var_t implements lcm.lcm.LCMEncodable
{
    public long timestamp;
    public short accel[];
    public int angle[];
    public int angular_vel[];
 
    public var_t()
    {
        accel = new short[3];
        angle = new int[3];
        angular_vel = new int[3];
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0x8f2f3ffe876c646aL;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class<?>>());
    }
 
    public static long _hashRecursive(ArrayList<Class<?>> classes)
    {
        if (classes.contains(lcmtypes.var_t.class))
            return 0L;
 
        classes.add(lcmtypes.var_t.class);
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
 
        for (int a = 0; a < 3; a++) {
            outs.writeShort(this.accel[a]); 
        }
 
        for (int a = 0; a < 3; a++) {
            outs.writeInt(this.angle[a]); 
        }
 
        for (int a = 0; a < 3; a++) {
            outs.writeInt(this.angular_vel[a]); 
        }
 
    }
 
    public var_t(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public var_t(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static lcmtypes.var_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        lcmtypes.var_t o = new lcmtypes.var_t();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.timestamp = ins.readLong();
 
        this.accel = new short[(int) 3];
        for (int a = 0; a < 3; a++) {
            this.accel[a] = ins.readShort();
        }
 
        this.angle = new int[(int) 3];
        for (int a = 0; a < 3; a++) {
            this.angle[a] = ins.readInt();
        }
 
        this.angular_vel = new int[(int) 3];
        for (int a = 0; a < 3; a++) {
            this.angular_vel[a] = ins.readInt();
        }
 
    }
 
    public lcmtypes.var_t copy()
    {
        lcmtypes.var_t outobj = new lcmtypes.var_t();
        outobj.timestamp = this.timestamp;
 
        outobj.accel = new short[(int) 3];
        System.arraycopy(this.accel, 0, outobj.accel, 0, 3); 
        outobj.angle = new int[(int) 3];
        System.arraycopy(this.angle, 0, outobj.angle, 0, 3); 
        outobj.angular_vel = new int[(int) 3];
        System.arraycopy(this.angular_vel, 0, outobj.angular_vel, 0, 3); 
        return outobj;
    }
 
}
