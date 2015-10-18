/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package lcmtypes;
 
import java.io.*;
import java.util.*;
import lcm.lcm.*;
 
public final class vicon_state_t implements lcm.lcm.LCMEncodable
{
    public long timestamp;
    public double position[];
    public double attitude[];
    public double velocity[];
    public double angular_vel[];
 
    public vicon_state_t()
    {
        position = new double[3];
        attitude = new double[3];
        velocity = new double[3];
        angular_vel = new double[3];
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0xdafc24cbdb24ced5L;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class<?>>());
    }
 
    public static long _hashRecursive(ArrayList<Class<?>> classes)
    {
        if (classes.contains(lcmtypes.vicon_state_t.class))
            return 0L;
 
        classes.add(lcmtypes.vicon_state_t.class);
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
            outs.writeDouble(this.position[a]); 
        }
 
        for (int a = 0; a < 3; a++) {
            outs.writeDouble(this.attitude[a]); 
        }
 
        for (int a = 0; a < 3; a++) {
            outs.writeDouble(this.velocity[a]); 
        }
 
        for (int a = 0; a < 3; a++) {
            outs.writeDouble(this.angular_vel[a]); 
        }
 
    }
 
    public vicon_state_t(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public vicon_state_t(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static lcmtypes.vicon_state_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        lcmtypes.vicon_state_t o = new lcmtypes.vicon_state_t();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.timestamp = ins.readLong();
 
        this.position = new double[(int) 3];
        for (int a = 0; a < 3; a++) {
            this.position[a] = ins.readDouble();
        }
 
        this.attitude = new double[(int) 3];
        for (int a = 0; a < 3; a++) {
            this.attitude[a] = ins.readDouble();
        }
 
        this.velocity = new double[(int) 3];
        for (int a = 0; a < 3; a++) {
            this.velocity[a] = ins.readDouble();
        }
 
        this.angular_vel = new double[(int) 3];
        for (int a = 0; a < 3; a++) {
            this.angular_vel[a] = ins.readDouble();
        }
 
    }
 
    public lcmtypes.vicon_state_t copy()
    {
        lcmtypes.vicon_state_t outobj = new lcmtypes.vicon_state_t();
        outobj.timestamp = this.timestamp;
 
        outobj.position = new double[(int) 3];
        System.arraycopy(this.position, 0, outobj.position, 0, 3); 
        outobj.attitude = new double[(int) 3];
        System.arraycopy(this.attitude, 0, outobj.attitude, 0, 3); 
        outobj.velocity = new double[(int) 3];
        System.arraycopy(this.velocity, 0, outobj.velocity, 0, 3); 
        outobj.angular_vel = new double[(int) 3];
        System.arraycopy(this.angular_vel, 0, outobj.angular_vel, 0, 3); 
        return outobj;
    }
 
}

