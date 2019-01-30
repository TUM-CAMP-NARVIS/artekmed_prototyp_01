__kernel void square(
        __global const uchar * src, int src_step, int src_offset,
        __global uchar * dstptr, int dst_step, int dst_offset, int dst_rows, int dst_cols)
{
    int x = get_global_id(0);
    int y = get_global_id(1);

    __global const float* data = (__global const float*)(src);
    __global float* result = (__global float*)(dstptr);

    if (x < dst_cols && y < dst_rows)
    {
        int src_index = mad24(y, src_step, src_offset + x);
        int dst_index = mad24(y, dst_step, dst_offset + x * (int)sizeof(float));

        result[dst_index] = data[src_index] * data[src_index];
    }
}