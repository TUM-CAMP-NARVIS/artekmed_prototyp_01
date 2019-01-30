__kernel void depthmap_to_points(
        __global const uchar * srcptr, int src_step, int src_offset,
        __global uchar * dstptr, int dst_step, int dst_offset, int rows, int cols,
        float ppx, float ppy, float fx, float fy,
        float depth_scale_factor)
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
                __global const uint16_t* src = (__global const uint16_t*)(srcptr + src_index);
                __global float* dst = (__global float*)(dstptr + dst_index);


                uint16_t z = src[0];
                if (z != 0) {
                    float depth = z * depth_scale_factor;




                    DATA_TYPE vx = (x - intrin(0,2)) / intrin(0,0);
                    DATA_TYPE vy = (y - intrin(1,2)) / intrin(1,1);
                    point(0) = depth * vx;
                    point(1) = depth * vy;
                    point(2) = depth;



// Map the top-left corner of the depth pixel onto the other image
Eigen::Vector2f depth_pixel(depth_x, depth_y);
Eigen::Vector3f depth_point;

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
        int dst_index = mad24(y, dst_step, dst_offset + x * (int)sizeof(float[3]));

{

}


float z = depth_img_rect.at<float>(depth_y, depth_x);
// Skip over depth pixels with the value of zero, we have no depth data so we will not write anything into our aligned images
if ((z != 0) && (!isnan(z)))
{


deproject_pixel_to_point(depth_point, intr_rect_ir, depth_pixel, depth);

// store pixel location
pt(0) = depth_point(0);
pt(1) = depth_point(1);
pt(2) = -depth_point(2);

cv::Vec4b pixel = color_img_rect.at<cv::Vec4b>(depth_y, depth_x);
colors[depth_pixel_index] = Eigen::Vector3f(pixel.val[2], pixel.val[1], pixel.val[0]) / 255.;
} else {
pt(0) = pt(1) = pt(2) = 0.;
}





        result[dst_index] = data[src_index] * data[src_index];
    }
}