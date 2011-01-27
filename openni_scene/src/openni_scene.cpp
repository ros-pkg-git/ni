/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010,
 *    Kei Okada <k-okada@jsk.t.u-tokyo.ac.jp>
 *    Yohei Kakiuchi <youhei@jsk.t.u-tokyo.ac.jp>
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Tokyo University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include <ros/ros.h>
#include <ros/package.h>

#include <XnOpenNI.h>
#include <XnCppWrapper.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

namespace my_ns
{
#if 0
  struct PointXYZRGBClass
 {
   float x;
   float y;
   float z;
   float rgb;
   uint32_t category;
  };
#else
  struct PointXYZRGBClass
  {
    PCL_ADD_POINT4D;  // This adds the members x,y,z which can also be accessed using the point (which is float[4])
    float rgb;
    uint32_t category;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  } EIGEN_ALIGN16;
#endif
} // namespace my_ns

POINT_CLOUD_REGISTER_POINT_STRUCT
(my_ns::PointXYZRGBClass,
 (float, x, x)
 (float, y, y)
 (float, z, z)
 (float, rgb, rgb)
 (uint32_t, category, category)
 );

//typedef pcl::PointXYZRGB PointType;
typedef my_ns::PointXYZRGBClass PointType;
typedef pcl::PointCloud<PointType> PointCloud;

using std::string;
xn::Context g_Context;
xn::ImageGenerator g_ImageGenerator;
xn::DepthGenerator g_DepthGenerator;
xn::SceneAnalyzer g_SceneAnalyzer;

//
ros::Publisher pub_depth_points2_;
ros::Publisher pub_rgb_;

// copy from openni.h
#define MAX(a,b) ((a)>(b)?(a):(b))
#define AVG(a,b) (((int)(a) + (int)(b)) >> 1)
#define AVG3(a,b,c) (((int)(a) + (int)(b) + (int)(c)) / 3)
#define AVG4(a,b,c,d) (((int)(a) + (int)(b) + (int)(c) + (int)(d)) >> 2)

#define CHECK_RC(nRetVal, what)                                         \
  if (nRetVal != XN_STATUS_OK){                                         \
    printf ("%s failed: %s\n", what, xnGetStatusString (nRetVal));      \
    return nRetVal;                                                     \
  }

// copy from OpenNIDriver::bayer2RGB
void bayer2RGB ( const xn::ImageMetaData& bayer, const xn::SceneMetaData& label, sensor_msgs::Image& image)
{
    register const XnUInt8 *bayer_pixel = bayer.Data();
    register unsigned yIdx, xIdx;

    // Assemble the color image data
    image.height = bayer.YRes();
    image.width = bayer.XRes();
    image.encoding = sensor_msgs::image_encodings::RGB8;
    image.data.resize (bayer.XRes() * bayer.YRes() * 3);
    image.step = bayer.XRes() * 3;


    int line_step = image.width;
    int line_step2 = image.width << 1;

    int rgb_line_step  = line_step * 3;             // previous color line
    register unsigned char *rgb_pixel = (unsigned char *)&image.data[0];

    {
      // first two pixel values for first two lines
      // Bayer         0 1 2
      //         0     G r g
      // line_step     b g b
      // line_step2    g r g
      rgb_pixel[3] = rgb_pixel[0] = bayer_pixel[1];    // red pixel
      rgb_pixel[1] = bayer_pixel[0];    // green pixel
      rgb_pixel[rgb_line_step + 2] = rgb_pixel[2] = bayer_pixel[line_step]; // blue;

      // Bayer         0 1 2
      //         0     g R g
      // line_step     b g b
      // line_step2    g r g
      //rgb_pixel[3] = bayer_pixel[1];
      rgb_pixel[4] = AVG3( bayer_pixel[0], bayer_pixel[2], bayer_pixel[line_step+1] );
      rgb_pixel[rgb_line_step + 5] = rgb_pixel[5] = AVG( bayer_pixel[line_step], bayer_pixel[line_step+2] );

      // BGBG line
      // Bayer         0 1 2
      //         0     g r g
      // line_step     B g b
      // line_step2    g r g
      rgb_pixel[rgb_line_step + 3] = rgb_pixel[rgb_line_step    ] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
      rgb_pixel[rgb_line_step + 1] = AVG3( bayer_pixel[0] , bayer_pixel[line_step+1] , bayer_pixel[line_step2] );
      //rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

      // pixel (1, 1)  0 1 2
      //         0     g r g
      // line_step     b G b
      // line_step2    g r g
      //rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
      rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
      //rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );

      rgb_pixel += 6;
      bayer_pixel += 2;
      // rest of the first two lines
      for (xIdx = 2; xIdx < image.width - 2; xIdx += 2, rgb_pixel += 6, bayer_pixel += 2)
      {
        // GRGR line
        // Bayer        -1 0 1 2
        //           0   r G r g
        //   line_step   g b g b
        // line_step2    r g r g
        rgb_pixel[0] = AVG( bayer_pixel[1], bayer_pixel[-1]);
        rgb_pixel[1] = bayer_pixel[0];
        rgb_pixel[2] = bayer_pixel[line_step + 1];

        // Bayer        -1 0 1 2
        //          0    r g R g
        //  line_step    g b g b
        // line_step2    r g r g
        rgb_pixel[3] = bayer_pixel[1];
        rgb_pixel[4] = AVG3( bayer_pixel[0], bayer_pixel[2], bayer_pixel[line_step+1] );
        rgb_pixel[rgb_line_step + 5] = rgb_pixel[5] = AVG( bayer_pixel[line_step], bayer_pixel[line_step+2] );

        // BGBG line
        // Bayer         -1 0 1 2
        //         0      r g r g
        // line_step      g B g b
        // line_step2     r g r g
        rgb_pixel[rgb_line_step    ] = AVG4( bayer_pixel[1] , bayer_pixel[line_step2+1], bayer_pixel[-1] , bayer_pixel[line_step2-1] );
        rgb_pixel[rgb_line_step + 1] = AVG4( bayer_pixel[0] , bayer_pixel[line_step2], bayer_pixel[line_step-1], bayer_pixel[line_step+1] );
        rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

        // Bayer         -1 0 1 2
        //         0      r g r g
        // line_step      g b G b
        // line_step2     r g r g
        rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
        rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
        //rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );
      }

      // last two pixel values for first two lines
      // GRGR line
      // Bayer        -1 0 1
      //           0   r G r
      //   line_step   g b g
      // line_step2    r g r
      rgb_pixel[0] = AVG( bayer_pixel[1], bayer_pixel[-1]);
      rgb_pixel[1] = bayer_pixel[0];
      rgb_pixel[rgb_line_step + 5] = rgb_pixel[rgb_line_step + 2] = rgb_pixel[5] = rgb_pixel[2] = bayer_pixel[line_step];

      // Bayer        -1 0 1
      //          0    r g R
      //  line_step    g b g
      // line_step2    r g r
      rgb_pixel[3] = bayer_pixel[1];
      rgb_pixel[4] = AVG( bayer_pixel[0], bayer_pixel[line_step+1] );
      //rgb_pixel[5] = bayer_pixel[line_step];

      // BGBG line
      // Bayer        -1 0 1
      //          0    r g r
      //  line_step    g B g
      // line_step2    r g r
      rgb_pixel[rgb_line_step    ] = AVG4( bayer_pixel[1] , bayer_pixel[line_step2+1], bayer_pixel[-1] , bayer_pixel[line_step2-1] );
      rgb_pixel[rgb_line_step + 1] = AVG4( bayer_pixel[0] , bayer_pixel[line_step2], bayer_pixel[line_step-1], bayer_pixel[line_step+1] );
      //rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

      // Bayer         -1 0 1
      //         0      r g r
      // line_step      g b G
      // line_step2     r g r
      rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
      rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
      //rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];

      bayer_pixel += line_step + 2;
      rgb_pixel += rgb_line_step + 6;

      // main processing
      for (yIdx = 2; yIdx < image.height-2; yIdx += 2)
      {
        // first two pixel values
        // Bayer         0 1 2
        //        -1     b g b
        //         0     G r g
        // line_step     b g b
        // line_step2    g r g

        rgb_pixel[3] = rgb_pixel[0] = bayer_pixel[1];    // red pixel
        rgb_pixel[1] = bayer_pixel[0];    // green pixel
        rgb_pixel[2] = AVG( bayer_pixel[line_step], bayer_pixel[-line_step] ); // blue;

        // Bayer         0 1 2
        //        -1     b g b
        //         0     g R g
        // line_step     b g b
        // line_step2    g r g
        //rgb_pixel[3] = bayer_pixel[1];
        rgb_pixel[4] = AVG4( bayer_pixel[0], bayer_pixel[2], bayer_pixel[line_step+1], bayer_pixel[1-line_step] );
        rgb_pixel[5] = AVG4( bayer_pixel[line_step], bayer_pixel[line_step+2], bayer_pixel[-line_step], bayer_pixel[2-line_step]);

        // BGBG line
        // Bayer         0 1 2
        //         0     g r g
        // line_step     B g b
        // line_step2    g r g
        rgb_pixel[rgb_line_step + 3] = rgb_pixel[rgb_line_step    ] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
        rgb_pixel[rgb_line_step + 1] = AVG3( bayer_pixel[0] , bayer_pixel[line_step+1] , bayer_pixel[line_step2] );
        rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

        // pixel (1, 1)  0 1 2
        //         0     g r g
        // line_step     b G b
        // line_step2    g r g
        //rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
        rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
        rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );

        rgb_pixel += 6;
        bayer_pixel += 2;
        // continue with rest of the line
        for (xIdx = 2; xIdx < image.width - 2; xIdx += 2, rgb_pixel += 6, bayer_pixel += 2)
        {
          // GRGR line
          // Bayer        -1 0 1 2
          //          -1   g b g b
          //           0   r G r g
          //   line_step   g b g b
          // line_step2    r g r g
          rgb_pixel[0] = AVG( bayer_pixel[1], bayer_pixel[-1]);
          rgb_pixel[1] = bayer_pixel[0];
          rgb_pixel[2] = AVG( bayer_pixel[line_step], bayer_pixel[-line_step] );

          // Bayer        -1 0 1 2
          //          -1   g b g b
          //          0    r g R g
          //  line_step    g b g b
          // line_step2    r g r g
          rgb_pixel[3] = bayer_pixel[1];
          rgb_pixel[4] = AVG4( bayer_pixel[0], bayer_pixel[2], bayer_pixel[line_step+1], bayer_pixel[1-line_step] );
          rgb_pixel[5] = AVG4( bayer_pixel[-line_step], bayer_pixel[2-line_step], bayer_pixel[line_step], bayer_pixel[line_step+2]);

          // BGBG line
          // Bayer         -1 0 1 2
          //         -1     g b g b
          //          0     r g r g
          // line_step      g B g b
          // line_step2     r g r g
          rgb_pixel[rgb_line_step    ] = AVG4( bayer_pixel[1], bayer_pixel[line_step2+1], bayer_pixel[-1], bayer_pixel[line_step2-1] );
          rgb_pixel[rgb_line_step + 1] = AVG4( bayer_pixel[0], bayer_pixel[line_step2], bayer_pixel[line_step-1], bayer_pixel[line_step+1] );
          rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

          // Bayer         -1 0 1 2
          //         -1     g b g b
          //          0     r g r g
          // line_step      g b G b
          // line_step2     r g r g
          rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
          rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
          rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );
        }

        // last two pixels of the line
        // last two pixel values for first two lines
        // GRGR line
        // Bayer        -1 0 1
        //           0   r G r
        //   line_step   g b g
        // line_step2    r g r
        rgb_pixel[0] = AVG( bayer_pixel[1], bayer_pixel[-1]);
        rgb_pixel[1] = bayer_pixel[0];
        rgb_pixel[rgb_line_step + 5] = rgb_pixel[rgb_line_step + 2] = rgb_pixel[5] = rgb_pixel[2] = bayer_pixel[line_step];

        // Bayer        -1 0 1
        //          0    r g R
        //  line_step    g b g
        // line_step2    r g r
        rgb_pixel[3] = bayer_pixel[1];
        rgb_pixel[4] = AVG( bayer_pixel[0], bayer_pixel[line_step+1] );
        //rgb_pixel[5] = bayer_pixel[line_step];

        // BGBG line
        // Bayer        -1 0 1
        //          0    r g r
        //  line_step    g B g
        // line_step2    r g r
        rgb_pixel[rgb_line_step    ] = AVG4( bayer_pixel[1] , bayer_pixel[line_step2+1], bayer_pixel[-1] , bayer_pixel[line_step2-1] );
        rgb_pixel[rgb_line_step + 1] = AVG4( bayer_pixel[0] , bayer_pixel[line_step2], bayer_pixel[line_step-1], bayer_pixel[line_step+1] );
        //rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

        // Bayer         -1 0 1
        //         0      r g r
        // line_step      g b G
        // line_step2     r g r
        rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
        rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
        //rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];

        bayer_pixel += line_step + 2;
        rgb_pixel += rgb_line_step + 6;
      }

      //last two lines
      // Bayer         0 1 2
      //        -1     b g b
      //         0     G r g
      // line_step     b g b

      rgb_pixel[rgb_line_step + 3] = rgb_pixel[rgb_line_step    ] = rgb_pixel[3] = rgb_pixel[0] = bayer_pixel[1];    // red pixel
      rgb_pixel[1] = bayer_pixel[0];    // green pixel
      rgb_pixel[rgb_line_step + 2] = rgb_pixel[2] = bayer_pixel[line_step]; // blue;

      // Bayer         0 1 2
      //        -1     b g b
      //         0     g R g
      // line_step     b g b
      //rgb_pixel[3] = bayer_pixel[1];
      rgb_pixel[4] = AVG4( bayer_pixel[0], bayer_pixel[2], bayer_pixel[line_step+1], bayer_pixel[1-line_step] );
      rgb_pixel[5] = AVG4( bayer_pixel[line_step], bayer_pixel[line_step+2], bayer_pixel[-line_step], bayer_pixel[2-line_step]);

      // BGBG line
      // Bayer         0 1 2
      //        -1     b g b
      //         0     g r g
      // line_step     B g b
      //rgb_pixel[rgb_line_step    ] = bayer_pixel[1];
      rgb_pixel[rgb_line_step + 1] = AVG( bayer_pixel[0] , bayer_pixel[line_step+1] );
      rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

      // Bayer         0 1 2
      //        -1     b g b
      //         0     g r g
      // line_step     b G b
      //rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
      rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
      rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );

      rgb_pixel += 6;
      bayer_pixel += 2;
      // rest of the last two lines
      for (xIdx = 2; xIdx < image.width - 2; xIdx += 2, rgb_pixel += 6, bayer_pixel += 2)
      {
        // GRGR line
        // Bayer       -1 0 1 2
        //        -1    g b g b
        //         0    r G r g
        // line_step    g b g b
        rgb_pixel[0] = AVG( bayer_pixel[1], bayer_pixel[-1]);
        rgb_pixel[1] = bayer_pixel[0];
        rgb_pixel[2] = AVG( bayer_pixel[line_step], bayer_pixel[-line_step]);

        // Bayer       -1 0 1 2
        //        -1    g b g b
        //         0    r g R g
        // line_step    g b g b
        rgb_pixel[rgb_line_step + 3] = rgb_pixel[3] = bayer_pixel[1];
        rgb_pixel[4] = AVG4( bayer_pixel[0], bayer_pixel[2], bayer_pixel[line_step+1], bayer_pixel[1-line_step] );
        rgb_pixel[5] = AVG4( bayer_pixel[line_step], bayer_pixel[line_step+2], bayer_pixel[-line_step], bayer_pixel[-line_step+2] );

        // BGBG line
        // Bayer       -1 0 1 2
        //        -1    g b g b
        //         0    r g r g
        // line_step    g B g b
        rgb_pixel[rgb_line_step    ] = AVG( bayer_pixel[-1], bayer_pixel[1] );
        rgb_pixel[rgb_line_step + 1] = AVG3( bayer_pixel[0], bayer_pixel[line_step-1], bayer_pixel[line_step+1] );
        rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];


        // Bayer       -1 0 1 2
        //        -1    g b g b
        //         0    r g r g
        // line_step    g b G b
        //rgb_pixel[rgb_line_step + 3] = bayer_pixel[1];
        rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
        rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );
      }

      // last two pixel values for first two lines
      // GRGR line
      // Bayer       -1 0 1
      //        -1    g b g
      //         0    r G r
      // line_step    g b g
      rgb_pixel[rgb_line_step    ] = rgb_pixel[0] = AVG( bayer_pixel[1], bayer_pixel[-1]);
      rgb_pixel[1] = bayer_pixel[0];
      rgb_pixel[5] = rgb_pixel[2] = AVG( bayer_pixel[line_step], bayer_pixel[-line_step]);

      // Bayer       -1 0 1
      //        -1    g b g
      //         0    r g R
      // line_step    g b g
      rgb_pixel[rgb_line_step + 3] = rgb_pixel[3] = bayer_pixel[1];
      rgb_pixel[4] = AVG3( bayer_pixel[0], bayer_pixel[line_step+1], bayer_pixel[-line_step+1] );
      //rgb_pixel[5] = AVG( bayer_pixel[line_step], bayer_pixel[-line_step] );

      // BGBG line
      // Bayer       -1 0 1
      //        -1    g b g
      //         0    r g r
      // line_step    g B g
      //rgb_pixel[rgb_line_step    ] = AVG2( bayer_pixel[-1], bayer_pixel[1] );
      rgb_pixel[rgb_line_step + 1] = AVG3( bayer_pixel[0] , bayer_pixel[line_step-1], bayer_pixel[line_step+1] );
      rgb_pixel[rgb_line_step + 5] = rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

      // Bayer       -1 0 1
      //        -1    g b g
      //         0    r g r
      // line_step    g b G
      //rgb_pixel[rgb_line_step + 3] = bayer_pixel[1];
      rgb_pixel[rgb_line_step + 4] = bayer_pixel[line_step+1];
      //rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];
    }
    {
      rgb_pixel = (unsigned char *)&image.data[0];
      register const XnLabel *label_pixel = label.Data();
      // filter processing
      for (yIdx = 0; yIdx < image.height; yIdx += 1)
        for (xIdx = 0; xIdx < image.width; xIdx += 1, rgb_pixel += 3, label_pixel++) {
	  if ( label_pixel[0] == 0  )
	    {
	      rgb_pixel[0] = rgb_pixel[1] = rgb_pixel[2] = 0;
	    }
	}
    }
}



// COPY from OpenNIDriver::publishXYZRGBPointCloud
void createScenePointCloud (const xn::DepthMetaData& dmd, const xn::SceneMetaData& smd,
			    const sensor_msgs::Image& rgb_image, PointCloud &cloud_out)
{
  cloud_out.height = dmd.YRes();
  cloud_out.width  = dmd.XRes();
  cloud_out.is_dense = false;

  //cloud_out.points.resize(cloud_out.height * cloud_out.width);
  cloud_out.points.resize(0);

  // copy from openni_driver.cpp
  union
  {
    struct /*anonymous*/
    {
      unsigned char Blue; // Blue channel
      unsigned char Green; // Green channel
      unsigned char Red; // Red channel
      unsigned char Alpha; // Alpha channel
    };
    float float_value;
    long long_value;
  } color;
  color.Alpha = 0;

  const XnDepthPixel* pDepth = dmd.Data();
  const XnLabel* pLabels = smd.Data();

  unsigned depth_step = dmd.XRes () / cloud_out.width;
  unsigned depth_skip = (dmd.YRes () / cloud_out.height - 1) * dmd.XRes ();

  int centerX = cloud_out.width >> 1;
  int centerY = cloud_out.height >> 1;

  int depth_idx = 0;
  int nLabel = 0;

  const double rgb_focal_length_VGA_ = 525;
  float constant = 0.001 / rgb_focal_length_VGA_;
  constant *= depth_step;

  unsigned char* rgb_buffer = (unsigned char*)&rgb_image.data[0];

  for (int v = 0; v < (int)cloud_out.height; ++v, depth_idx += depth_skip)
  {
    for (int u = 0; u < (int)cloud_out.width; ++u, depth_idx += depth_step)
    {
      //pcl::PointXYZRGB& pt = cloud_out(u, v);
      PointType pt;

      if ( pLabels[depth_idx] > 0 ) {
	nLabel = MAX( nLabel, pLabels[depth_idx] );
	// Fill in XYZ
	pt.x = (u - centerX) * pDepth[depth_idx] * constant;
	pt.y = (v - centerY) * pDepth[depth_idx] * constant;
	pt.z = pDepth[depth_idx] * 0.001;

	color.Red   = rgb_buffer[depth_idx*3+0];
	color.Green = rgb_buffer[depth_idx*3+1];
	color.Blue  = rgb_buffer[depth_idx*3+2];
	pt.rgb = color.float_value;
	pt.category= pLabels[depth_idx];

	cloud_out.points.push_back(pt);
      }
    }
  }
  ROS_INFO("found %d people", nLabel);
}

int main (int argc, char **argv)
{
    ros::init (argc, argv, "openni_scene");
    ros::NodeHandle nh("~");

    pub_depth_points2_ = nh.advertise<PointCloud >("scene",15);
    pub_rgb_ = nh.advertise<sensor_msgs::Image> ("rgb/image_color", 15);

    // Initialize OpenNI
    string configFilename = ros::package::getPath ("openni_scene") + "/openni_scene.xml";
    XnStatus nRetVal = g_Context.InitFromXmlFile (configFilename.c_str ());
    CHECK_RC (nRetVal, "InitFromXml");

    nRetVal = g_Context.FindExistingNode (XN_NODE_TYPE_IMAGE, g_ImageGenerator);
    CHECK_RC (nRetVal, "Find image generator");
    nRetVal = g_Context.FindExistingNode (XN_NODE_TYPE_DEPTH, g_DepthGenerator);
    CHECK_RC (nRetVal, "Find depth generator");
    nRetVal = g_Context.FindExistingNode (XN_NODE_TYPE_SCENE, g_SceneAnalyzer);
    CHECK_RC (nRetVal, "Find scene analyzer");

    // Initialization done. Start generating
    nRetVal = g_Context.StartGeneratingAll ();
    CHECK_RC (nRetVal, "StartGenerating");

    // Grayscale: bypass debayering -> gives us bayer pattern!
#if 0 // add <PixelFormat>Grayscale8</PixelFormat> openni_scene.xml for kinect
    if (g_ImageGenerator.SetPixelFormat(XN_PIXEL_FORMAT_GRAYSCALE_8_BIT )!= XN_STATUS_OK)
      {
	ROS_ERROR("[openni_scene] Failed to set image pixel format to 8bit-grayscale");
	return (false);
      }
#endif

    while (ros::ok ())
    {
        g_Context.WaitAndUpdateAll ();
	ros::Time time = ros::Time::now();

	xn::ImageMetaData imageMD;
	xn::SceneMetaData sceneMD;
	xn::DepthMetaData depthMD;
	g_ImageGenerator.GetMetaData(imageMD);
	g_DepthGenerator.GetMetaData(depthMD);
	g_SceneAnalyzer.GetMetaData(sceneMD);

	sensor_msgs::Image rgb_image;
	bayer2RGB( imageMD, sceneMD, rgb_image);

	if (pub_rgb_.getNumSubscribers() > 0)
	  {
	    rgb_image.header.stamp    = time;
	    rgb_image.header.frame_id = "openni_rgb_optical_frame";
	    rgb_image.header.seq      = imageMD.FrameID();

	    pub_rgb_.publish (rgb_image);
	  }
	if (pub_depth_points2_.getNumSubscribers() > 0  )
	  {
	    PointCloud cloud_out;
	    cloud_out.header.stamp = time;
	    cloud_out.header.frame_id = "openni_depth_optical_frame";
	    cloud_out.header.seq = depthMD.FrameID();

	    createScenePointCloud (depthMD, sceneMD, rgb_image, cloud_out);

	    //pub_depth_points2_.publish (cloud_out.makeShared ());
	    cloud_out.width = cloud_out.points.size();
	    if (cloud_out.width > 0) {
	      cloud_out.height = 1;
	      cloud_out.is_dense = true;
	      pub_depth_points2_.publish (cloud_out.makeShared ());
	    }
	  }
	//ros::spinOnce();
    }
    g_Context.Shutdown ();
    return 0;
}
