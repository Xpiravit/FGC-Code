package org.firstinspires.ftc.utilities;

import java.util.ArrayList;

/**Implements a basic RingBuffer with ArrayLists
 * Useful for keeping track of error - time pairs etc.
 * Find more information about Ring Buffers <a href="https://en.wikipedia.org/wiki/Circular_buffer">here</a>
 * @author gabkion
 *  */
public class RingBuffer<E>{
    protected int index = 0;
    protected ArrayList<E> buffer;

    public RingBuffer(int length, E startingValue)
    {
        buffer = new ArrayList<>(length);

        for (int i = 0; i < length; i++) buffer.add(startingValue);
    }

    public E getValue(E newValue)
    {
        E retVal = buffer.get(index);
        buffer.set(index, newValue);

        index++;
        index = index % buffer.size();

        return retVal;
    }
}
