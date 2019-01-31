__kernel void depthmap_to_points(
        __global const uchar * srcptr, int src_step, int src_offset,
        __global uchar * dstptr, int dst_step, int dst_offset, int rows, int cols,
        float depth_ppx, float depth_ppy, float depth_fx, float depth_fy, float depth_scale_factor)
{
    int x = get_global_id(0);
    int y = get_global_id(1) * PIX_PER_WI_Y;

    if (x < cols)
    {
        int src_index = mad24(y, src_step, mad24(x, scnbytes, src_offset));
        int dst_index = mad24(y, dst_step, mad24(x, dcnbytes, dst_offset));

#pragma unroll
        for (int cy = 0; cy < PIX_PER_WI_Y; ++cy)
        {
            if (y < rows)
            {
                __global const INPUT_TYPE* src = (__global const INPUT_TYPE*)(srcptr + src_index);
                __global OUTPUT_TYPE* dst = (__global OUTPUT_TYPE*)(dstptr + dst_index);


                INPUT_TYPE z = src[0];
                if (z != 0) {
                    DATA_TYPE depth = z * depth_scale_factor;

                    DATA_TYPE vx = (x - intrin(0,2)) / intrin(0,0);
                    DATA_TYPE vy = (y - intrin(1,2)) / intrin(1,1);

                    dst[0] = -depth * vx;
                    dst[1] = -depth * vy;
                    dst[2] = -depth;

                } else {
                    dst[0] = dst[1] = dst[2] = 0.;
                }

                ++y;
                dst_index += dst_step;
                src_index += src_step;
            }
        }
    }
}