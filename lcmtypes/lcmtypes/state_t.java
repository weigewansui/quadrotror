/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package lcmtypes;
 
import java.io.*;
import java.util.*;
import lcm.lcm.*;
 
public final class state_t implements lcm.lcm.LCMEncodable
{
    public long timestamp;
    public int position[];
    public int velocity[];
    public short accel[];
    public int angle[];
    public int angular_vel[];
    public int angular_accel[];
    public short rpm[];
 
    public state_t()
    {
        position = new int[3];
        velocity = new int[3];
        accel = new short[3];
        angle = new int[3];
        angular_vel = new int[3];
        angular_accel = new int[3];
        rpm = new short[4];
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0x17dbcbe9aecc16d0L;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class<?>>());
    }
 
    public static long _hashRecursive(ArrayList<Class<?>> classes)
    {
        if (classes.contains(lcmtypes.state_t.class))
            return 0L;
 
        classes.add(lcmtypes.state_t.class);
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
            outs.writeInt(this.position[a]); 
        }
 
        for (int a = 0; a < 3; a++) {
            outs.writeInt(this.velocity[a]); 
        }
 
        for (int a = 0; a < 3; a++) {
            outs.writeShort(this.accel[a]); 
        }
 
        for (int a = 0; a < 3; a++) {
            outs.writeInt(this.angle[a]); 
        }
 
        for (int a = 0; a < 3; a++) {
            outs.writeInt(this.angular_vel[a]); 
        }
 
        for (int a = 0; a < 3; a++) {
            outs.writeInt(this.angular_accel[a]); 
        }
 
        for (int a = 0; a < 4; a++) {
            outs.writeShort(this.rpm[a]); 
        }
 
    }
 
    public state_t(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public state_t(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static lcmtypes.state_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        lcmtypes.state_t o = new lcmtypes.state_t();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.timestamp = ins.readLong();
 
        this.position = new int[(int) 3];
        for (int a = 0; a < 3; a++) {
            this.position[a] = ins.readInt();
        }
 
        this.velocity = new int[(int) 3];
        for (int a = 0; a < 3; a++) {
            this.velocity[a] = ins.readInt();
        }
 
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
 
        this.angular_accel = new int[(int) 3];
        for (int a = 0; a < 3; a++) {
            this.angular_accel[a] = ins.readInt();
        }
 
        this.rpm = new short[(int) 4];
        for (int a = 0; a < 4; a++) {
            this.rpm[a] = ins.readShort();
        }
 
    }
 
    public lcmtypes.state_t copy()
    {
        lcmtypes.state_t outobj = new lcmtypes.state_t();
        outobj.timestamp = this.timestamp;
 
        outobj.position = new int[(int) 3];
        System.arraycopy(this.position, 0, outobj.position, 0, 3); 
        outobj.velocity = new int[(int) 3];
        System.arraycopy(this.velocity, 0, outobj.velocity, 0, 3); 
        outobj.accel = new short[(int) 3];
        System.arraycopy(this.accel, 0, outobj.accel, 0, 3); 
        outobj.angle = new int[(int) 3];
        System.arraycopy(this.angle, 0, outobj.angle, 0, 3); 
        outobj.angular_vel = new int[(int) 3];
        System.arraycopy(this.angular_vel, 0, outobj.angular_vel, 0, 3); 
        outobj.angular_accel = new int[(int) 3];
        System.arraycopy(this.angular_accel, 0, outobj.angular_accel, 0, 3); 
        outobj.rpm = new short[(int) 4];
        System.arraycopy(this.rpm, 0, outobj.rpm, 0, 4); 
        return outobj;
    }
 
}
