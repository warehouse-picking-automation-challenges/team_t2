/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Toshiba Corporation, 
 *                     Toshiba Infrastructure Systems & Solutions Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Toshiba Corporation, nor the Toshiba
 *       Infrastructure Systems & Solutions Corporation, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include "robot_vision_recognition2.h"

// Include message type
#include <T2_robot_vision/RecognizedItem.h>

#include <ros/package.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "opencv2/core/core_c.h"
#include "opencv2/flann/miniflann.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/photo/photo.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/ml/ml.hpp"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/contrib/contrib.hpp"
#include <ros/console.h>
#include <log4cxx/logger.h>

#include <iterator>
#include <set>
#include <cstdio>
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>


//#define DEBUG

#define TH_SEGLINE_MICHI 80
#define MODE_USE_HD_ROI
#define MODE_INPUTGT_SEGAREA
//#define MODE_INPUTGT_CATID

#define MODE_MATCH_ALLAREA 1 //run all VGA -> refer segimg's area
#define INVALID_DEPTH_VALUE 10

#define ROUND_CIRCLE 5
#define SCORE_INVALID_SEGMENT 5

#define TH_RATE_OVERLAP 0.10
#define TH_RATE_CHAMPION_INCLUDED 0.85
#define MINUS_SIMILARITY_VALUE 2

#define WATCH_ONE_OBJECT

#define TH_CNT_SAME_HUE 30  //40
#define TH_RANGE_SAME_HUE 15

//detect white
#define TH_SATURATION_WHITE_LOWERTHAN 40
#define TH_RANGE_SAME_VALUE 15

std::string objlist[127]{
    "none" ,      //0
    "none",      //1
    "none",      //2
    "none",      //3
    "none",      //4
    "none",      //5
    "none",      //6
    "none",      //7
    "none",      //8
    "none",      //9
    "none",      //10
    "none",      //11
    "none",      //12
    "none",      //13
    "none",      //14
    "none",      //15
    "none",      //16
    "none",      //17
    "none",      //18
    "none",      //19
    "none",      //20
    "none",      //21
    "none",      //22
    "none",      //23
    "none",      //24
    "none",      //25
    "none",      //26
    "none",      //27
    "none",      //28
    "none",      //29
    "none",      //30
    "none",      //31
    "none",      //32
    "none",      //33
    "none",      //34
    "none",      //35
    "none",      //36
    "none",      //37
    "none",      //38
    "none",      //39
    "none",      //40
    "none",      //41
    "none",      //42
    "none",      //43
    "none",      //44
    "none",      //45
    "none",      //46
    "none",      //47
    "none",      //48
    "none",      //49
    "none",      //50
    "none",      //51
    "none",      //52
    "none",      //53
    "none",      //54
    "none",      //55
    "none",      //56
    "none",      //57
    "none",      //58
    "none",      //59
    "none",      //60
    "none",      //61
    "avery_binder",      //62
    "balloons",      //63
    "band_aid_tape",      //64
    "#bath_sponge",      //65
    "#black_fashion_gloves",      //66
    "burts_bees_baby_wipes",      //67
    "colgate_toothbrush_4pk",      //68
    "composition_book",      //69
    "crayons",      //70
    "duct_tape",      //71
    "epsom_salts",      //72
    "expo_eraser",      //73
    "fiskars_scissors",      //74
    "flashlight",      //75
    "glue_sticks",      //76
    "hand_weight",      //77
    "hanes_socks",      //78
    "hinged_ruled_index_cards",      //79
    "ice_cube_tray",      //80
    "irish_spring_soap",      //81
    "laugh_out_loud_jokes",      //82
    "##marbles",      //83
    "measuring_spoon",      //84
    "##mesh_cup",      //85
    "mouse_traps",      //86
    "pie_plates",      //87
    "##plastic_wine_glass",      //88
    "##poland_spring_water",      //89
    "reynolds_wrap",      //90
    "robots_dvd",      //91
    "robots_everywhere",      //92
    "scotch_sponges",      //93
    "speed_stick",      //94
    "#table_cloth",      //95
    "tennis_ball_container",      //96
    "ticonderoga_pencils",      //97
    "tissue_box",      //98
    "toilet_brush",      //99
    "#white_facecloth",      //100
    "##windex",      //101
    "none",      //102
    "none",      //103
    "none",      //104
    "none",      //105
    "none",      //106
    "none",      //107
    "none",      //108
    "none",      //109
    "none",      //110
    "new1",      //111
    "new2",      //112
    "new3",      //113
    "new4",      //114
    "new5",      //115
    "new6",      //116
    "new7",      //117
    "new8",      //118
    "new9",      //119
    "new10",      //120
    "new11",      //121
    "new12",      //122
    "new13",      //123
    "new14",      //124
    "new15",      //125
    "new16",      //126
};

// Register nodelet
PLUGINLIB_EXPORT_CLASS(T2_robot_vision::robot_vision_recognition2, nodelet::Nodelet);

std::string int_to_string(int n)
{
    std::stringstream ss;
    ss << n;
    std::string result = ss.str();

    return result;
}

namespace T2_robot_vision
{

robot_vision_recognition2::robot_vision_recognition2()
{
}

/*
    * Nodelet Destructor.
    */
robot_vision_recognition2::~robot_vision_recognition2()
{
}

// Function prototypes
void subtractPlane(const cv::Mat& depth, cv::Mat& mask, std::vector<CvPoint>& chain, double f);

std::vector<CvPoint> maskFromTemplate(const std::vector<cv::linemod::Template>& templates,
                                      int num_modalities, cv::Point offset, cv::Size size,
                                      cv::Mat& mask, cv::Mat& dst);

void templateConvexHull(const std::vector<cv::linemod::Template>& templates,
                        int num_modalities, cv::Point offset, cv::Size size,
                        cv::Mat& dst);

void drawResponse(const std::vector<cv::linemod::Template>& templates,
                  int num_modalities, cv::Mat& dst, cv::Point offset, int T);

cv::Mat displayQuantized(const cv::Mat& quantized);
// Copy of cv_mouse from cv_utilities

class Mouse
{
public:
    static void start(const std::string& a_img_name)
    {
        cvSetMouseCallback(a_img_name.c_str(), Mouse::cv_on_mouse, 0);
    }
    static int event(void)
    {
        int l_event = m_event;
        m_event = -1;
        return l_event;
    }
    static int x(void)
    {
        return m_x;
    }
    static int y(void)
    {
        return m_y;
    }

private:
    static void cv_on_mouse(int a_event, int a_x, int a_y, int, void *)
    {
        m_event = a_event;
        m_x = a_x;
        m_y = a_y;
    }

    static int m_event;
    static int m_x;
    static int m_y;
};
int Mouse::m_event;
int Mouse::m_x;
int Mouse::m_y;

// Adapted from cv_timer in cv_utilities
class Timer
{
public:
    Timer() : start_(0), time_(0) {}

    void start()
    {
        start_ = cv::getTickCount();
    }

    void stop()
    {
        CV_Assert(start_ != 0);
        int64 end = cv::getTickCount();
        time_ += end - start_;
        start_ = 0;
    }

    double time()
    {
        double ret = time_ / cv::getTickFrequency();
        time_ = 0;
        return ret;
    }

private:
    int64 start_, time_;
};

// Functions to store detector and templates in single XML/YAML file
static cv::Ptr<cv::linemod::Detector> readLinemod(const std::string& filename)
{

    cv::Ptr<cv::linemod::Detector> detector = new cv::linemod::Detector;
    //std::cout << "Line : " << __LINE__ << std::endl;
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    //std::cout << "Line : " << __LINE__ << std::endl;

    detector->read(fs.root());

    cv::FileNode fn = fs["classes"];

    for (cv::FileNodeIterator i = fn.begin(), iend = fn.end(); i != iend ; ++i)
    {
        detector->readClass(*i);
    }

    ROS_INFO_STREAM("successfully imported : " << filename);

    return detector;
}

static void writeLinemod(const cv::Ptr<cv::linemod::Detector>& detector, const std::string& filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    detector->write(fs);

    std::vector<std::string> ids = detector->classIds();
    fs << "classes" << "[";
    for (int i = 0; i < (int)ids.size(); ++i)
    {
        fs << "{";
        detector->writeClass(ids[i], fs);
        fs << "}"; // current class
    }
    fs << "]"; // classes
}

void paste(cv::Mat dst, cv::Mat src, int x, int y, int width, int height) {
    cv::Mat resized_img;
    cv::resize(src, resized_img, cv::Size(width, height));

    if (x >= dst.cols || y >= dst.rows) return;
    int w = (x >= 0) ? std::min(dst.cols - x, resized_img.cols) : std::min(std::max(resized_img.cols + x, 0), dst.cols);
    int h = (y >= 0) ? std::min(dst.rows - y, resized_img.rows) : std::min(std::max(resized_img.rows + y, 0), dst.rows);
    int u = (x >= 0) ? 0 : std::min(-x, resized_img.cols - 1);
    int v = (y >= 0) ? 0 : std::min(-y, resized_img.rows - 1);
    int px = std::max(x, 0);
    int py = std::max(y, 0);

    cv::Mat roi_dst = dst(cv::Rect(px, py, w, h));
    cv::Mat roi_resized = resized_img(cv::Rect(u, v, w, h));
    roi_resized.copyTo(roi_dst);
}

void paste(cv::Mat dst, cv::Mat src, int x, int y) {
    paste(dst, src, x, y, src.rows, src.cols);
}

static void reprojectPoints(const std::vector<cv::Point3d>& proj, std::vector<cv::Point3d>& real, double f)
{

    real.resize(proj.size());
    double f_inv = 1.0 / f;

    for (int i = 0; i < (int)proj.size(); ++i)
    {
        double Z = proj[i].z;
        real[i].x = (proj[i].x - 320.) * (f_inv * Z);
        real[i].y = (proj[i].y - 240.) * (f_inv * Z);
        real[i].z = Z;
    }

}

static void filterPlane(IplImage * ap_depth, std::vector<IplImage *> & a_masks, std::vector<CvPoint> & a_chain, double f)
{

    const int l_num_cost_pts = 200;

    float l_thres = 4;

    IplImage * lp_mask = cvCreateImage(cvGetSize(ap_depth), IPL_DEPTH_8U, 1);
    cvSet(lp_mask, cvRealScalar(0));

    std::vector<CvPoint> l_chain_vector;

    float l_chain_length = 0;
    float * lp_seg_length = new float[a_chain.size()];

    for (int l_i = 0; l_i < (int)a_chain.size(); ++l_i)
    {
        float x_diff = (float)(a_chain[(l_i + 1) % a_chain.size()].x - a_chain[l_i].x);
        float y_diff = (float)(a_chain[(l_i + 1) % a_chain.size()].y - a_chain[l_i].y);
        lp_seg_length[l_i] = sqrt(x_diff*x_diff + y_diff*y_diff);
        l_chain_length += lp_seg_length[l_i];
    }
    for (int l_i = 0; l_i < (int)a_chain.size(); ++l_i)
    {
        if (lp_seg_length[l_i] > 0)
        {
            int l_cur_num = cvRound(l_num_cost_pts * lp_seg_length[l_i] / l_chain_length);
            float l_cur_len = lp_seg_length[l_i] / l_cur_num;

            for (int l_j = 0; l_j < l_cur_num; ++l_j)
            {
                float l_ratio = (l_cur_len * l_j / lp_seg_length[l_i]);

                CvPoint l_pts;

                l_pts.x = cvRound(l_ratio * (a_chain[(l_i + 1) % a_chain.size()].x - a_chain[l_i].x) + a_chain[l_i].x);
                l_pts.y = cvRound(l_ratio * (a_chain[(l_i + 1) % a_chain.size()].y - a_chain[l_i].y) + a_chain[l_i].y);

                l_chain_vector.push_back(l_pts);
            }
        }
    }
    std::vector<cv::Point3d> lp_src_3Dpts(l_chain_vector.size());

    for (int l_i = 0; l_i < (int)l_chain_vector.size(); ++l_i)
    {
        lp_src_3Dpts[l_i].x = l_chain_vector[l_i].x;
        lp_src_3Dpts[l_i].y = l_chain_vector[l_i].y;
        lp_src_3Dpts[l_i].z = CV_IMAGE_ELEM(ap_depth, unsigned short, cvRound(lp_src_3Dpts[l_i].y), cvRound(lp_src_3Dpts[l_i].x));
        //CV_IMAGE_ELEM(lp_mask,unsigned char,(int)lp_src_3Dpts[l_i].Y,(int)lp_src_3Dpts[l_i].X)=255;
    }
    //cv_show_image(lp_mask,"hallo2");

    reprojectPoints(lp_src_3Dpts, lp_src_3Dpts, f);

    CvMat * lp_pts = cvCreateMat((int)l_chain_vector.size(), 4, CV_32F);
    CvMat * lp_v = cvCreateMat(4, 4, CV_32F);
    CvMat * lp_w = cvCreateMat(4, 1, CV_32F);

    for (int l_i = 0; l_i < (int)l_chain_vector.size(); ++l_i)
    {
        CV_MAT_ELEM(*lp_pts, float, l_i, 0) = (float)lp_src_3Dpts[l_i].x;
        CV_MAT_ELEM(*lp_pts, float, l_i, 1) = (float)lp_src_3Dpts[l_i].y;
        CV_MAT_ELEM(*lp_pts, float, l_i, 2) = (float)lp_src_3Dpts[l_i].z;
        CV_MAT_ELEM(*lp_pts, float, l_i, 3) = 1.0f;
    }
    cvSVD(lp_pts, lp_w, 0, lp_v);

    float l_n[4] = { CV_MAT_ELEM(*lp_v, float, 0, 3),
                     CV_MAT_ELEM(*lp_v, float, 1, 3),
                     CV_MAT_ELEM(*lp_v, float, 2, 3),
                     CV_MAT_ELEM(*lp_v, float, 3, 3) };

    float l_norm = sqrt(l_n[0] * l_n[0] + l_n[1] * l_n[1] + l_n[2] * l_n[2]);

    l_n[0] /= l_norm;
    l_n[1] /= l_norm;
    l_n[2] /= l_norm;
    l_n[3] /= l_norm;

    float l_max_dist = 0;

    for (int l_i = 0; l_i < (int)l_chain_vector.size(); ++l_i)
    {
        float l_dist = l_n[0] * CV_MAT_ELEM(*lp_pts, float, l_i, 0) +
                l_n[1] * CV_MAT_ELEM(*lp_pts, float, l_i, 1) +
                l_n[2] * CV_MAT_ELEM(*lp_pts, float, l_i, 2) +
                l_n[3] * CV_MAT_ELEM(*lp_pts, float, l_i, 3);

        if (fabs(l_dist) > l_max_dist)
            l_max_dist = l_dist;
    }
    //std::cerr << "plane: " << l_n[0] << ";" << l_n[1] << ";" << l_n[2] << ";" << l_n[3] << " maxdist: " << l_max_dist << " end" << std::endl;
    int l_minx = ap_depth->width;
    int l_miny = ap_depth->height;
    int l_maxx = 0;
    int l_maxy = 0;

    for (int l_i = 0; l_i < (int)a_chain.size(); ++l_i)
    {
        l_minx = std::min(l_minx, a_chain[l_i].x);
        l_miny = std::min(l_miny, a_chain[l_i].y);
        l_maxx = std::max(l_maxx, a_chain[l_i].x);
        l_maxy = std::max(l_maxy, a_chain[l_i].y);
    }
    int l_w = l_maxx - l_minx + 1;
    int l_h = l_maxy - l_miny + 1;
    int l_nn = (int)a_chain.size();

    CvPoint * lp_chain = new CvPoint[l_nn];

    for (int l_i = 0; l_i < l_nn; ++l_i)
        lp_chain[l_i] = a_chain[l_i];

    cvFillPoly(lp_mask, &lp_chain, &l_nn, 1, cvScalar(255, 255, 255));

    delete[] lp_chain;

    std::vector<cv::Point3d> lp_dst_3Dpts(l_h * l_w);

    int l_ind = 0;

    for (int l_r = 0; l_r < l_h; ++l_r)
    {
        for (int l_c = 0; l_c < l_w; ++l_c)
        {
            lp_dst_3Dpts[l_ind].x = l_c + l_minx;
            lp_dst_3Dpts[l_ind].y = l_r + l_miny;
            lp_dst_3Dpts[l_ind].z = CV_IMAGE_ELEM(ap_depth, unsigned short, l_r + l_miny, l_c + l_minx);
            ++l_ind;
        }
    }

    reprojectPoints(lp_dst_3Dpts, lp_dst_3Dpts, f);

    l_ind = 0;

    for (int l_r = 0; l_r < l_h; ++l_r)
    {
        for (int l_c = 0; l_c < l_w; ++l_c)
        {
            float l_dist = (float)(l_n[0] * lp_dst_3Dpts[l_ind].x + l_n[1] * lp_dst_3Dpts[l_ind].y + lp_dst_3Dpts[l_ind].z * l_n[2] + l_n[3]);

            ++l_ind;

            if (CV_IMAGE_ELEM(lp_mask, unsigned char, l_r + l_miny, l_c + l_minx) != 0)
            {
                if (fabs(l_dist) < std::max(l_thres, (l_max_dist * 2.0f)))
                {
                    for (int l_p = 0; l_p < (int)a_masks.size(); ++l_p)
                    {
                        int l_col = cvRound((l_c + l_minx) / (l_p + 1.0));
                        int l_row = cvRound((l_r + l_miny) / (l_p + 1.0));

                        CV_IMAGE_ELEM(a_masks[l_p], unsigned char, l_row, l_col) = 0;
                    }
                }
                else
                {
                    for (int l_p = 0; l_p < (int)a_masks.size(); ++l_p)
                    {
                        int l_col = cvRound((l_c + l_minx) / (l_p + 1.0));
                        int l_row = cvRound((l_r + l_miny) / (l_p + 1.0));

                        CV_IMAGE_ELEM(a_masks[l_p], unsigned char, l_row, l_col) = 255;
                    }
                }
            }
        }
    }


    cvReleaseImage(&lp_mask);
    cvReleaseMat(&lp_pts);
    cvReleaseMat(&lp_w);
    cvReleaseMat(&lp_v);
}

void subtractPlane(const cv::Mat& depth, cv::Mat& mask, std::vector<CvPoint>& chain, double f)
{
    mask = cv::Mat::zeros(depth.size(), CV_8U);
    std::vector<IplImage*> tmp;
    IplImage mask_ipl = mask;
    tmp.push_back(&mask_ipl);
    IplImage depth_ipl = depth;
    filterPlane(&depth_ipl, tmp, chain, f);
}

std::vector<CvPoint> maskFromTemplate(const std::vector<cv::linemod::Template>& templates,
                                      int num_modalities, cv::Point offset, cv::Size size,
                                      cv::Mat& mask, cv::Mat& dst)
{
    templateConvexHull(templates, num_modalities, offset, size, mask);

    const int OFFSET = 30;
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), OFFSET);

    CvMemStorage * lp_storage = cvCreateMemStorage(0);
    CvTreeNodeIterator l_iterator;
    CvSeqReader l_reader;
    CvSeq * lp_contour = 0;

    cv::Mat mask_copy = mask.clone();
    IplImage mask_copy_ipl = mask_copy;
    cvFindContours(&mask_copy_ipl, lp_storage, &lp_contour, sizeof(CvContour),
                   CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

    std::vector<CvPoint> l_pts1; // to use as input to cv_primesensor::filter_plane

    cvInitTreeNodeIterator(&l_iterator, lp_contour, 1);
    while ((lp_contour = (CvSeq *)cvNextTreeNode(&l_iterator)) != 0)
    {
        CvPoint l_pt0;
        cvStartReadSeq(lp_contour, &l_reader, 0);
        CV_READ_SEQ_ELEM(l_pt0, l_reader);
        l_pts1.push_back(l_pt0);

        for (int i = 0; i < lp_contour->total; ++i)
        {
            CvPoint l_pt1;
            CV_READ_SEQ_ELEM(l_pt1, l_reader);
            /// @todo Really need dst at all? Can just as well do this outside
            cv::line(dst, l_pt0, l_pt1, CV_RGB(0, 255, 0), 2);

            l_pt0 = l_pt1;
            l_pts1.push_back(l_pt0);
        }
    }
    cvReleaseMemStorage(&lp_storage);

    return l_pts1;
}

// Adapted from cv_show_angles
cv::Mat displayQuantized(const cv::Mat& quantized)
{
    cv::Mat color(quantized.size(), CV_8UC3);
    for (int r = 0; r < quantized.rows; ++r)
    {
        const uchar* quant_r = quantized.ptr(r);
        cv::Vec3b* color_r = color.ptr<cv::Vec3b>(r);

        for (int c = 0; c < quantized.cols; ++c)
        {
            cv::Vec3b& bgr = color_r[c];
            switch (quant_r[c])
            {
            case 0:   bgr[0] = 0; bgr[1] = 0; bgr[2] = 0;    break;
            case 1:   bgr[0] = 55; bgr[1] = 55; bgr[2] = 55;    break;
            case 2:   bgr[0] = 80; bgr[1] = 80; bgr[2] = 80;    break;
            case 4:   bgr[0] = 105; bgr[1] = 105; bgr[2] = 105;    break;
            case 8:   bgr[0] = 130; bgr[1] = 130; bgr[2] = 130;    break;
            case 16:  bgr[0] = 155; bgr[1] = 155; bgr[2] = 155;    break;
            case 32:  bgr[0] = 180; bgr[1] = 180; bgr[2] = 180;    break;
            case 64:  bgr[0] = 205; bgr[1] = 205; bgr[2] = 205;    break;
            case 128: bgr[0] = 230; bgr[1] = 230; bgr[2] = 230;    break;
            case 255: bgr[0] = 0; bgr[1] = 0; bgr[2] = 255;    break;
            default:  bgr[0] = 0; bgr[1] = 255; bgr[2] = 0;    break;
            }
        }
    }
    return color;
}

// Adapted from cv_line_template::convex_hull
void templateConvexHull(const std::vector<cv::linemod::Template>& templates,
                        int num_modalities, cv::Point offset, cv::Size size,
                        cv::Mat& dst)
{
    std::vector<cv::Point> points;
    for (int m = 0; m < num_modalities; ++m)
    {
        for (int i = 0; i < (int)templates[m].features.size(); ++i)
        {
            cv::linemod::Feature f = templates[m].features[i];
            points.push_back(cv::Point(f.x, f.y) + offset);
        }
    }

    std::vector<cv::Point> hull;
    cv::convexHull(points, hull);

    dst = cv::Mat::zeros(size, CV_8U);


    const int hull_count = (int)hull.size();
    const cv::Point* hull_pts = &hull[0];
    cv::fillPoly(dst, &hull_pts, &hull_count, 1, cv::Scalar(255));
//    cv::imshow("dst",dst);
//    cv::waitKey();

}

void drawResponse(const std::vector<cv::linemod::Template>& templates,
                  int num_modalities, cv::Mat& dst, cv::Point offset, int T)
{
    static const cv::Scalar COLORS[5] = { CV_RGB(0, 0, 255),
                                          CV_RGB(0, 255, 0),
                                          CV_RGB(255, 255, 0),
                                          CV_RGB(255, 140, 0),
                                          CV_RGB(255, 0, 0) };

    for (int m = 1; m < num_modalities; ++m)
    {
        cv::Scalar color = COLORS[m];

        for (int i = 0; i < (int)templates[m].features.size(); ++i)
        {
            cv::linemod::Feature f = templates[m].features[i];
            cv::Point pt(f.x + offset.x, f.y + offset.y);

            int line_length = 8;
            cv::Point2i pt2(line_length * cos(f.label*0.395) + pt.x, line_length * sin(f.label*0.395) + pt.y);
            cv::Point2i pt3(-line_length * cos(f.label*0.395) + pt.x, -line_length * sin(f.label*0.395) + pt.y);

            cv::circle(dst, pt, T / 2, color);
            cv::putText(dst, int_to_string((int)f.label), pt, 0, 0.4, cv::Scalar(0,0,0), 2, 8);
            cv::putText(dst, int_to_string((int)f.label), pt, 0, 0.4, cv::Scalar(255,255,255), 1, 8);
        }
    }
}

int checkNormBriefness(const std::vector<cv::linemod::Template>& templates,
                  int num_modalities, cv::Mat& dst, cv::Point offset, int T)
{
    static const cv::Scalar COLORS[5] = { CV_RGB(0, 0, 255),
                                          CV_RGB(0, 255, 0),
                                          CV_RGB(255, 255, 0),
                                          CV_RGB(255, 140, 0),
                                          CV_RGB(255, 0, 0) };

    int hist_norm[8] = {0,0,0,0,0,0,0,0};
    int num_norm = 0;

    for (int m = 1; m < num_modalities; ++m)
    {
        cv::Scalar color = COLORS[m];

        for (int i = 0; i < (int)templates[m].features.size(); ++i)
        {
            cv::linemod::Feature f = templates[m].features[i];


            if(((int)f.label >= 0)&&((int)f.label < 8))
            {
                hist_norm[(int)f.label]++;
                num_norm++;
            }

        }

        if(num_norm == 63)
        {
            int max_hist_norm = 0;
            int second_max_hist_norm = 0;
            int idx_max_hist_norm = 0;

            for(int i=0; i<8; i++)
            {
                if(max_hist_norm < hist_norm[i] )
                {
                    max_hist_norm = hist_norm[i];
                    idx_max_hist_norm = i;

                    second_max_hist_norm = max_hist_norm;
                }

            }

            int flag_use = 1;

            if(templates[m].width < 100)
                flag_use = 0;

            if(templates[m].height < 100)
                flag_use = 0;

            if(max_hist_norm > 45)
                flag_use = 0;

            return flag_use;

        }
        else
        {
            return 0;
        }
    }
}

int checkDaenScore(const std::vector<cv::linemod::Template>& templates,
                  int num_modalities, cv::Mat& dst, cv::Point offset, int T)
{
    static const cv::Scalar COLORS[5] = { CV_RGB(0, 0, 255),
                                          CV_RGB(0, 255, 0),
                                          CV_RGB(255, 255, 0),
                                          CV_RGB(255, 140, 0),
                                          CV_RGB(255, 0, 0) };


    double daenScore_output=0;

    for (int m = 1; m < num_modalities; ++m)
    {
        cv::Scalar color = COLORS[m];
#define N_POINTS (64)
        cv::Mat X = cv::Mat::zeros(cv::Size(2,N_POINTS), CV_64F);
        for(int it_fp=0; it_fp < N_POINTS; it_fp++ )
        {
            cv::linemod::Feature f = templates[m].features[it_fp];
            X.at<double>(it_fp,0) = (double)f.x / 100.0;
            X.at<double>(it_fp,1) = (double)f.y / 100.0;
        }
        cv::Mat M = cv::Mat::zeros(cv::Size(2,1), CV_64F);
        for(int it_fp=0; it_fp < N_POINTS; it_fp++ )
        {
            M.at<double>(0) += X.at<double>(it_fp,0)/N_POINTS;
            M.at<double>(1) += X.at<double>(it_fp,1)/N_POINTS;
        }
        for(int it_fp=0; it_fp < N_POINTS; it_fp++ )
        {
            X.at<double>(it_fp,0) -= M.at<double>(0);
            X.at<double>(it_fp,1) -= M.at<double>(1);
        }
        cv::Mat C = cv::Mat::zeros(cv::Size(2,2), CV_64F);
        for(int it_fp=0; it_fp < N_POINTS; it_fp++ )
        {
            C.at<double>(0,0) += X.at<double>(it_fp,0)*X.at<double>(it_fp,0) / N_POINTS ;
            C.at<double>(1,0) += X.at<double>(it_fp,1)*X.at<double>(it_fp,0) / N_POINTS ;
            C.at<double>(0,1) += X.at<double>(it_fp,0)*X.at<double>(it_fp,1) / N_POINTS ;
            C.at<double>(1,1) += X.at<double>(it_fp,1)*X.at<double>(it_fp,1) / N_POINTS ;
        }

        cv::Mat EVec;
        cv::Mat EVal;
        cv::eigen(C,EVal,EVec);
        EVal.at<double>(0)  = pow(EVal.at<double>(0),0.5);
        EVal.at<double>(1)  = pow(EVal.at<double>(1),0.5);
        double daenScore = EVal.at<double>(1) / (EVal.at<double>(0)+1e-8);
        daenScore_output = daenScore;
    }

#define TH_DAEN_SCORE (0.0)
    if(daenScore_output > TH_DAEN_SCORE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

void createMaskFromFeaturePoints(const std::vector<cv::linemod::Template>& templates,
                  int num_modalities, cv::Mat& dst, cv::Point offset, int T)
{

    dst = cv::Mat::zeros(dst.size(),CV_8UC1);

    for (int m = 0; m < num_modalities; ++m)
    {

        for (int i = 1; i < (int)templates[m].features.size(); ++i)
        {
            cv::linemod::Feature f_prev = templates[m].features[i-1];
            cv::Point pt_prev(f_prev.x + offset.x, f_prev.y + offset.y);

            cv::linemod::Feature f = templates[m].features[i];
            cv::Point pt(f.x + offset.x, f.y + offset.y);

            cv::line(dst, pt_prev, pt, cv::Scalar(255), 3,CV_AA);
        }
    }

    cv::dilate(dst, dst, cv::Mat(), cv::Point(-1, -1), 12);
    cv::erode(dst, dst, cv::Mat(), cv::Point(-1, -1), 5);

    cv::Rect rect(offset.x, offset.y, templates[0].width, templates[0].height);//size_srcimg.width, size_srcimg.height);

    cv::Mat dst_roi = cv::Mat(dst, rect).clone();

    dst = dst_roi.clone();


}

int countMatchedTemplateSize(const std::vector<cv::linemod::Template>& templates,
                             int num_modalities, cv::Mat& dst, cv::Point offset, int T)
{

    int counterMatchedTemplatePixels = 0;

    cv::Mat rgb = dst.clone();
    rgb = rgb * 0;
    for (int m = 0; m < num_modalities; ++m)
    {
        for (int i = 0; i < (int)templates[m].features.size(); ++i)
        {
            cv::linemod::Feature f = templates[m].features[i];
            cv::Point pt(f.x + offset.x, f.y + offset.y);
            cv::circle(rgb, pt, T, CV_RGB(255, 255, 255),-1);
        }
    }
    //cv::imshow("dst", dst);

    cv::Mat gray;
    cv::cvtColor(rgb, gray, CV_RGB2GRAY);
    //cv::erode(gray, gray, cv::Mat(), cv::Point(-1, -1), 5);

    for(int y=0; y < gray.rows; y++)
        for(int x=0; x < gray.cols; x++)
        {
            if(gray.at<unsigned char>(y,x) == 255)
            {
                counterMatchedTemplatePixels++;
            }
        }

    //cv::imshow("gray", gray);
    //cv::waitKey();

    return counterMatchedTemplatePixels;
}

class TempInfo
{
public:
    int check;
    std::string class_id;
    std::string lightpattern;
    int object;
    int yaw;
    int pitch;
    std::string power;
    int roll;
    int tlx;
    int tly;
    int width;
    int height;
    int tempsize;
};

std::vector<TempInfo> importTempInfoData(std::string filename)
{
    std::ifstream file;
    file.open(filename.c_str());
    std::string wow, mem, key, waste;
    unsigned int x = 0;

    std::vector<TempInfo> temps;
    std::getline(file, waste);

    while (true) {
        int counter = 0;
        TempInfo readdata;

        std::getline(file, wow);
        if (file.fail()) break; //check for error

        while (x <= wow.length()) {
            if (wow[x] == ',' || wow[x] == '\0') {
                int temp = 0;
                switch (counter)
                {
                case 0:
                    temp = atoi(mem.c_str());
                    readdata.check = temp;
                    break;
                case 1:
                    readdata.class_id = mem;
                    break;
                case 2:
                    readdata.lightpattern = mem;
                    break;
                case 3:
                    temp = atoi(mem.c_str());
                    readdata.object = temp;
                    break;
                case 4:
                    temp = atoi(mem.c_str());
                    readdata.yaw = temp;
                    break;
                case 5:
                    temp = atoi(mem.c_str());
                    readdata.pitch = temp;
                    break;
                case 6:
                    //temp = atof(mem.c_str());
                    //readdata.power = temp;
                    readdata.power = mem;
                    break;
                case 7:
                    temp = atoi(mem.c_str());
                    readdata.roll = temp;
                    break;
                case 8:
                    temp = atoi(mem.c_str());
                    readdata.tlx = temp;
                    break;
                case 9:
                    temp = atoi(mem.c_str());
                    readdata.tly = temp;
                    break;
                case 10:
                    temp = atoi(mem.c_str());
                    readdata.width = temp;
                    break;
                case 11:
                    temp = atoi(mem.c_str());
                    readdata.height = temp;
                    break;
                case 12:
                    temp = atoi(mem.c_str());
                    readdata.tempsize = temp;
                    break;
                default:
                    break;
                }

                key = mem;
                mem.clear();
                x++; //step over ','
                counter++;
            }
            else
                mem += wow[x++];
        }

        temps.push_back(readdata);
        //list_map0[key] = mem; //char to string
        //list_map1[mem] = key; //string to char
        mem.clear(); //reset memory
        x = 0;//reset index
    }

    //ROS_INFO("LineMOD : %d templates\n", (int)temps.size());
    ROS_INFO_STREAM("successfully imported :" << filename);
    file.close();
    return temps;
}

struct Recog2Prm{
    //mode flags
    int mode_size_inputimg;
    int mode_rotate_srcimg;
    int mode_save_result_image;
    int mode_change_depth_range;
    int mode_main_recognition2;
    int threshold_matching_score;
    float penalty_coefficient;
    float threshold_segment_vs_template;
    int round_circle;

    //rotate step input image
    int step_roll;

    Recog2Prm()
    {
        mode_size_inputimg = -1;
        mode_rotate_srcimg = -1;
        step_roll = 360;
        mode_save_result_image = -1;
        round_circle = 5;

        mode_change_depth_range = -1;
        mode_main_recognition2 = -1;
        threshold_matching_score = -1;

        penalty_coefficient = -1.0;
        threshold_segment_vs_template = -1.0;
    }

    bool Checker(void)
    {
        if (mode_size_inputimg < 0){ return false; }
        if (mode_rotate_srcimg < 0){ return false; }
        if (mode_save_result_image < 0){ return false; }
        if (mode_change_depth_range < 0){ return false; }
        if (mode_main_recognition2 < 0){ return false; }
        if (step_roll == 0){ return false; }
        if (!((0 <= threshold_matching_score) && (threshold_matching_score <= 100))){ return false; }
        if (penalty_coefficient < (float)0.0){

            std::cout << penalty_coefficient << "<" << (float)0.0 << std::endl;

            return false; }
        //if (threshold_segment_vs_template < 1.0){ return false; }
        return true;
    }
};

std::ostream& operator<<(std::ostream& os, Recog2Prm const &recog2_prm)
{
    return os << "mode_main_recognition2 : " << recog2_prm.mode_main_recognition2 << std::endl
              << "mode_size_inputimg : " << recog2_prm.mode_size_inputimg << std::endl
              << "mode_rotate_srcimg : " << recog2_prm.mode_rotate_srcimg << std::endl
              << "mode_save_result_image : " << recog2_prm.mode_save_result_image << std::endl
              << "mode_change_depth_range : " << recog2_prm.mode_change_depth_range << std::endl
              << "step_roll : " << recog2_prm.step_roll << std::endl
              << "threshold_matching_score : " << recog2_prm.threshold_matching_score << std::endl
              << "penalty_coefficient : " << recog2_prm.penalty_coefficient << std::endl
              << "threshold_segment_vs_template : " << recog2_prm.threshold_segment_vs_template << std::endl;
}

static void Recog2PrmFileRead(std::string file_path, std::string camera_name, Recog2Prm& recog2_prm)
{
    ROS_DEBUG_STREAM("T2_robot_vision:recognition2 " << __FUNCTION__ << " start");

    std::ifstream f_st;
    ROS_DEBUG_STREAM("T2_robot_vision:recognition2 parameter file path" << file_path);

    f_st.open(file_path.c_str(), std::ios::in);
    if (!f_st)
    {
        ROS_ERROR_STREAM("T2_robot_vision:recognition2 parameter file read error:" << file_path);
        recog2_prm.mode_main_recognition2 = -1;
        ROS_DEBUG_STREAM("T2_robot_vision:recognition2 " << __FUNCTION__ << " end");
        return;
    }
    while (!f_st.eof())
    {
        std::string line_buf;
        std::getline(f_st, line_buf);
        const char comment_mark = '#';
        const char delim_mark = ':';
        if (line_buf.find(comment_mark) == 0)
        {
            continue;
        }
        std::string::size_type p = line_buf.find(delim_mark);
        if (p == 0 || p == std::string::basic_string::npos)
        {
            continue;
        }
        std::string title = line_buf.substr(0, p);
        std::string t_value = line_buf.substr(p + 1, std::string::basic_string::npos);

        //std::cout << title << ":" << t_value << std::endl;

        if (title == "mode_main_recognition2")
        {
            recog2_prm.mode_main_recognition2 = atoi(t_value.c_str());
            std::cout << title << ":" << recog2_prm.mode_main_recognition2 << std::endl;
            continue;
        }
        if (title == "mode_size_inputimg")
        {
            recog2_prm.mode_size_inputimg = atoi(t_value.c_str());
            std::cout << title << ":" << recog2_prm.mode_size_inputimg << std::endl;
            continue;
        }
        if (title == "mode_save_result_image")
        {
            recog2_prm.mode_save_result_image = atoi(t_value.c_str());
            std::cout << title << ":" << recog2_prm.mode_save_result_image << std::endl;
            continue;
        }
        if (title == "mode_change_depth_range")
        {
            recog2_prm.mode_change_depth_range = atoi(t_value.c_str());
            std::cout << title << ":" << recog2_prm.mode_change_depth_range << std::endl;
            continue;
        }
        if (title == "mode_rotate_srcimg")
        {
            recog2_prm.mode_rotate_srcimg = atoi(t_value.c_str());
            std::cout << title << ":" << recog2_prm.mode_rotate_srcimg << std::endl;
            continue;
        }
        if (title == "step_roll")
        {
            recog2_prm.step_roll = atoi(t_value.c_str());
            std::cout << title << ":" << recog2_prm.step_roll << std::endl;
            continue;
        }
        if (title == "threshold_matching_score")
        {
            recog2_prm.threshold_matching_score = atoi(t_value.c_str());
            std::cout << title << ":" << recog2_prm.threshold_matching_score << std::endl;
            continue;
        }
        if (title == "penalty_coefficient")
        {
            recog2_prm.penalty_coefficient = atof(t_value.c_str());
            std::cout << title << ":" << recog2_prm.penalty_coefficient << std::endl;
            continue;
        }
        if (title == "threshold_segment_vs_template")
        {
            recog2_prm.threshold_segment_vs_template = atof(t_value.c_str());
            std::cout << title << ":" << recog2_prm.threshold_segment_vs_template << std::endl;
            continue;
        }


        if (title == "round_circle")
        {
            recog2_prm.round_circle = atoi(t_value.c_str());
            std::cout << title << ":" << recog2_prm.round_circle << std::endl;
            continue;
        }
    }

#ifdef CHANGE_DEPTH_RANGE
    recog2_prm.mode_change_depth_range = 1;
#endif

#ifndef CHANGE_DEPTH_RANGE
    recog2_prm.mode_change_depth_range = 0;
#endif

    if (!recog2_prm.Checker())
    {
        recog2_prm.mode_main_recognition2 = 0;
        ROS_ERROR_STREAM("Error : invalid values in recog2_prm or recog2_prm.txt.");
    }
    ROS_DEBUG_STREAM("T2_robot_vision:recognition2 " << __FUNCTION__ << " end");
}

std::vector< std::vector<cv::Mat> > rotate_img(cv::Mat color, cv::Mat depth, cv::Point gp, int step_rotate)
{

    std::vector<std::vector<cv::Mat> > rotated_sources;
    float scale = 1.0;

    //clockwise
    for (float angle = 0.0; angle < 360.0; angle += (float)step_rotate)
    {

        //define rotate matrix
        cv::Mat matrix = cv::getRotationMatrix2D(gp, angle, scale);

        std::vector<cv::Mat> sources;
        cv::Mat rotated_color;
        cv::Mat rotated_depth;

        //rotate img
        warpAffine(color, rotated_color, matrix, rotated_color.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0));
        warpAffine(depth, rotated_depth, matrix, rotated_depth.size(), cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(INVALID_DEPTH_VALUE));

        sources.push_back(rotated_color);
        sources.push_back(rotated_depth);

        rotated_sources.push_back(sources);

        //        cv::namedWindow("color", CV_WINDOW_AUTOSIZE);
        //        cv::imshow("color", rotated_color);
        //        cv::namedWindow("depth", CV_WINDOW_AUTOSIZE);
        //        cv::imshow("depth", rotated_depth);
        //        cv::waitKey(0);
    }

    return rotated_sources;
}

cv::Mat rotate_img(cv::Mat color, cv::Point gp, int angle)
{

    float scale = 1.0;
    cv::Mat rotated_color;

    cv::Mat matrix = cv::getRotationMatrix2D(gp, angle, scale);
    //rotate img
    warpAffine(color, rotated_color, matrix, rotated_color.size());

    return rotated_color;
}

class MatchResult
{
public:
    int int_class_id;
    float similarity;
    int maskid;
    int roll;
    int tlx;
    int tly;
    int brx;
    int bry;

    std::string class_id;
    TempInfo temp;
    cv::Mat mask;
};

//import file definition
cv::Ptr<cv::linemod::Detector> detector = nullptr;
std::vector<TempInfo> TemplatesData;

//std::string filename_match_result_csv;
std::ofstream ofs_matchres("matching_result_cam0.csv");

/*
    * Initialize the nodelet.
*/

void robot_vision_recognition2::onInit()
{
    ROS_INFO_STREAM("onInit(): T2_robot_vision: recognition2 begin");

    ros::NodeHandle &nh = getNodeHandle();
    ros::NodeHandle &pnh = getPrivateNodeHandle();

    std::string src_root_path = ros::package::getPath("T2_robot_vision");

    //get config from recog2prm.txt
    Recog2Prm recog2_prm;
    std::string prm_file_path = src_root_path + "/data/recog2_prm.txt";
    std::string camera_name = "sr300";  //camera_name used because the parameters are named such as "sr300_...." in recog2_prm.txt

    Recog2PrmFileRead(prm_file_path, camera_name, recog2_prm);

    //SELECT Dummy Output Mode
    if (recog2_prm.mode_main_recognition2 == 0)
    {

    }
    else
    {

        //VGA
        if(recog2_prm.mode_size_inputimg == 1)
        {

            //get tempdatas from templates.yml
            detector = readLinemod(src_root_path + "/data/linemod_templates_VGA.yml");
            std::vector<std::string> ids = detector->classIds();

            if (ids.empty())
            {
                ROS_ERROR("robot_vision_recognition2: couldn't read templates_VGA.yml\n");
                //std::copy(ids.begin(), ids.end(), std::ostream_iterator<std::string>(std::cout, "\n"));
            }

            //get other necessary information from template.csv
            std::string filenamecsv = src_root_path + "/data/checklist_template_existence_VGA.csv";

            if(TemplatesData.size() == 0)
                TemplatesData = importTempInfoData(filenamecsv);

            if (TemplatesData.empty())
            {
                ROS_ERROR("robot_vision_recognition2: couldn't read checklist_template_existence_VGA.csv\n");
            }
        }

        //HD
        if(recog2_prm.mode_size_inputimg == 0)
        {

            //get tempdatas from templates.yml

            if(detector == nullptr)
            {
                detector = readLinemod(src_root_path + "/data/linemod_templates_HD.yml");
            }
            std::vector<std::string> ids = detector->classIds();

            if (ids.empty())
            {
                ROS_ERROR("robot_vision_recognition2: couldn't read templates_HD.yml\n");
                //std::copy(ids.begin(), ids.end(), std::ostream_iterator<std::string>(std::cout, "\n"));
            }

            //get other necessary information from template.csv

if(TemplatesData.size()==0)
{
            std::string filenamecsv = src_root_path + "/data/checklist_template_existence_HD.csv";
            TemplatesData = importTempInfoData(filenamecsv);
}

            if (TemplatesData.empty())
            {
                ROS_ERROR("robot_vision_recognition2: couldn't read checklist_template_existence_HD.csv\n");
            }

        }
    }

    ofs_matchres << "jobno,seg_id,score,category,yaw,pitch,roll" << std::endl;

    //inputimg_counter = -1;

    recognition_yolo_sub_ = nh.subscribe("yolov2_results", 10, &robot_vision_recognition2::resultYOLOv2Callback, this);
    recognition_seg_sub_ = nh.subscribe("recognized_segment", 10, &robot_vision_recognition2::recognizedSegmentCallback, this);
    recognition_yolo_pub_ = nh.advertise<T2_robot_vision::RecognizedItem>("recognized_item", 10);
    recognition_seg_pub_ = nh.advertise<T2_robot_vision::RecognizedItem>("recognized_item", 10);
    recognition_seg_2plane_pub_ = nh.advertise<T2_robot_vision::RecognizedItem>("LineMOD_results", 10);

    ROS_INFO_STREAM("onInit(): T2_robot_vision: recognition2 end");
}


void checkOverlapBetweenTemplates(int* p_idx_champion,
                                  std::vector<int>* p_idx_overlaps,
                                  std::vector< cv::linemod::Match > matches,
                                  std::vector< TempInfo > TemplatesData)
{

    cv::Mat disp = cv::Mat::zeros(480, 640, CV_8UC1);


    cv::linemod::Match champion = matches[*p_idx_champion];
    std::vector< cv::linemod::Match > candidates = matches;
    candidates.erase(candidates.begin());

//    std::cout << matches.size() << " -> " << candidates.size() << std::endl;

//    std::cout << "first idx:" << *p_idx_champion << std::endl;
    (*p_idx_overlaps).push_back(*p_idx_champion);

    //std::vector<int> idx_overlap;
    //(*p_idx_overlaps).push_back(0);

    for(int i=0; i < candidates.size(); i++)
    {
        int classid_champion = atoi((champion.class_id).substr(5).c_str());
        cv::Rect rect_champion(champion.x, champion.y, TemplatesData[classid_champion].width, TemplatesData[classid_champion].height);
        int size_rect_champion = rect_champion.width * rect_champion.height;
        int th_overlapsize = size_rect_champion * TH_RATE_OVERLAP;

        int classid_candidate = atoi((candidates[i].class_id).substr(5).c_str());
        cv::Rect rect_candidate(candidates[i].x, candidates[i].y
                                , TemplatesData[classid_candidate].width, TemplatesData[classid_candidate].height);
        int size_rect_candidate = rect_candidate.width * rect_candidate.height;

        cv::Rect rect_overlap = rect_champion & rect_candidate;
        int size_rect_overlap = rect_overlap.width * rect_overlap.height;

        // if overlap area is large, erase from candidates
        if(size_rect_overlap > th_overlapsize)
        {
            //std::cout << "idx:" << i+1 << std::endl;
            (*p_idx_overlaps).push_back(i+1);

            cv::rectangle(disp, rect_candidate, 50, 1);

            int  size_most_of_rect_champion = size_rect_champion * TH_RATE_CHAMPION_INCLUDED;
            //std::cout << i << "include rate ? : " << size_rect_overlap << " vs " << size_most_of_rect_champion << std::endl;

            if(size_rect_overlap > size_most_of_rect_champion)
            {
                if(size_rect_candidate > size_rect_champion)
                {
                    //std::cout << i << "sym comparison ? : " << candidates[i].similarity << " vs " << champion.similarity << std::endl;

                    if(candidates[i].similarity > champion.similarity - MINUS_SIMILARITY_VALUE)
                    {
                        cv::rectangle(disp, rect_champion, 50, 1);

                        *p_idx_champion = i;

                        candidates[i].similarity = champion.similarity;

                        //cv::linemod::Match match_cushion = champion;
                        champion = candidates[i];
                        //candidates[i] = match_cushion;

                        //cv::Rect rect_cushion = rect_champion;
                        rect_champion = rect_candidate;
                        //rect_candidate = rect_cushion;
                        cv::rectangle(disp, rect_champion, 255, 1);
                    }
                }
            }
        }
        else
        {
            cv::rectangle(disp, rect_candidate, 150, 1);
        }

        cv::rectangle(disp, rect_champion, 255, 1);
//        cv::imshow("disp", disp);
        //cv::waitKey(0);
    }

    return ;
}


int countTemplatePointOnSegment(cv::Mat segment, const std::vector<cv::linemod::Template>& templates,
                  int num_modalities, cv::Point offset)
{

    int num_overlap_point = 0;

    //int num_of_point = 0;

    //cv::imshow("dddd", segment);

    for (int m = 0; m < num_modalities; ++m)
    {
        for (int i = 0; i < (int)templates[m].features.size(); ++i)
        {
            cv::linemod::Feature f = templates[m].features[i];
            cv::Point pt(f.x + offset.x, f.y + offset.y);
            //num_of_point++ ;
            //cv::circle(segment, pt, 5 / 2, cv::Scalar(100));


            if(segment.at<uchar>(pt.y, pt.x) == 255)
            {
                num_overlap_point++;
            }
        }
    }

    return num_overlap_point;
}

int countPixelsBetweenTwoValues(cv::Mat src_img, int th_low, int th_high)
{

    int num_of_point = 0;

    for(int y=0; y < src_img.rows; y++){
        for(int x=0; x < src_img.cols; x++)
        {
            if( th_low <= src_img.at<unsigned char>(y,x) && src_img.at<unsigned char>(y,x) <= th_high)
            {
                num_of_point++;
            }
        }
    }

    //std::cout << num_of_point << std::endl;

    return num_of_point;
}


std::vector<int> checkOverlapWithSegments(std::vector< cv::linemod::Match > matches,
                              std::vector< cv::Mat > segments,
                              std::vector< TempInfo > TemplatesData)
{
    std::vector<int> list_idx_best_segment;

    for (int itr_match = 0; itr_match < matches.size(); itr_match++)
    {
        cv::Mat display_comp_with_segment = cv::Mat::zeros(480, 640, CV_8UC3);
        const std::vector<cv::linemod::Template>& templates = detector->getTemplates(matches[itr_match].class_id, matches[itr_match].template_id);
        drawResponse(templates, 2, display_comp_with_segment, cv::Point(matches[itr_match].x, matches[itr_match].y), detector->getT(0));
//        cv::imshow("display", display_comp_with_segment);
//        cv::waitKey();

        std::vector<int> list_temppoints_on_segment;
        std::vector<int> list_whitepoints_on_segment;

        for(int idx_segment = 0; idx_segment < segments.size(); idx_segment++)
        {
//            cv::imshow("segments", segments[idx_segment]);
//            cv::waitKey();

            /** calc overlap rate**/

            // 1) count template point on the segment
            int temppoints_on_segment = countTemplatePointOnSegment(segments[idx_segment], templates, 2, cv::Point(matches[itr_match].x, matches[itr_match].y));
            list_temppoints_on_segment.push_back(temppoints_on_segment);
            //std::cout << idx_segment << ":" << temppoints_on_segment << std::endl;

            // 2) count white pixels on segment
            int whitepoints_on_segment = countPixelsBetweenTwoValues(segments[idx_segment], 255, 255);
            list_whitepoints_on_segment.push_back(whitepoints_on_segment);
            //std::cout << idx_segment << ":" << whitepoints_on_segment << std::endl;
        }

        /** TODO : have to decide TH rate & num of pixel **/
        int max_temppoints_on_segment = 126*0.85;

        std::vector<int> list_idx_candidate_segment;

        // comp with temppoint on segment
        for(int idx_segment = 0; idx_segment < segments.size(); idx_segment++)
        {
            if(list_temppoints_on_segment[idx_segment] > max_temppoints_on_segment)
            {
                max_temppoints_on_segment = list_temppoints_on_segment[idx_segment];
                list_idx_candidate_segment.clear();
                list_idx_candidate_segment.push_back(idx_segment);
            }
            else if(list_temppoints_on_segment[idx_segment] == max_temppoints_on_segment)
            {
                list_idx_candidate_segment.push_back(idx_segment);
            }
            else
            {
                // Do Nothing
            }
        }

        if(list_idx_candidate_segment.size() == 1)
        {
            //could find only one best segment
            list_idx_best_segment.push_back(list_idx_candidate_segment[0]);
        }
        else if(list_idx_candidate_segment.size() > 1)
        {
            //there is more than 1 candidate, so comp with other prm( white point(mask valid pixel) on segment )

            int idx_segment_min_whitepoints = list_idx_candidate_segment[0];

            for(int idx = 1; idx < list_idx_candidate_segment.size(); idx++)
            {
                if(list_whitepoints_on_segment[list_idx_candidate_segment[idx]] < list_whitepoints_on_segment[idx_segment_min_whitepoints])
                {
                    idx_segment_min_whitepoints = list_idx_candidate_segment[idx];
                }
            }

            list_idx_best_segment.push_back(idx_segment_min_whitepoints);
        }
        else
        {
            //couldn't find good segment
            list_idx_best_segment.push_back(-1);
        }

    }

    return list_idx_best_segment;
}


std::vector< cv::linemod::Match > checkOverlapWithOtherResult(std::vector< cv::linemod::Match > matches,
                                                              std::vector<MatchResult>  v_matchRes,
                                                              std::vector< cv::Mat > mMat_v,
                                                              std::vector< TempInfo > TemplatesData)
{
//    std::vector<int> list_idx_best_segment;

    cv::Mat src_segment = mMat_v[0].clone();

    for(int y=0; y< src_segment.rows; y++)
        for(int x=0; x<src_segment.cols; x++)
            src_segment.at<uchar>(y, x) = 0;

    for(int i = 0; i < v_matchRes.size(); i++)
    {
        for(int y=0; y< v_matchRes[i].mask.rows; y++)
            for(int x=0; x<v_matchRes[i].mask.cols; x++)
            {
                if(v_matchRes[i].mask.at<uchar>(y, x) > 0)
                    src_segment.at<uchar>(v_matchRes[i].tly + y, v_matchRes[i].tlx + x) = 255;
            }
    }

//    dilate(src_segment)
//    cv::imshow("src_segment", src_segment);
//    cv::waitKey();
//    cv::dilate(src_segment, src_segment, cv::Mat(), cv::Point(-1, -1), 20);
//    cv::imshow("src_segment", src_segment);
//    cv::waitKey();

    int th_pixels_ovlap = 126 * 0.15;
    int itr_match = 0;

    while(matches.size() > 0)
    {
        cv::Mat display_comp_with_segment = cv::Mat::zeros(480, 640, CV_8UC3);
        const std::vector<cv::linemod::Template>& templates = detector->getTemplates(matches[itr_match].class_id, matches[itr_match].template_id);
//        drawResponse(templates, 2, display_comp_with_segment, cv::Point(matches[itr_match].x, matches[itr_match].y), detector->getT(0));
//        cv::imshow("display", display_comp_with_segment);
//        cv::waitKey();

       /** At first check the match[n] in the foreground **/
        int cnt_inside_the_forground = countTemplatePointOnSegment(mMat_v[0], templates, 2, cv::Point(matches[itr_match].x, matches[itr_match].y));

        int pixels_ovlap = countTemplatePointOnSegment(src_segment, templates, 2, cv::Point(matches[itr_match].x, matches[itr_match].y));
        /// result area tono kaburiga chiisai tokidake jikkou
        if((pixels_ovlap < th_pixels_ovlap)&&(cnt_inside_the_forground < 126*0.80))
        {

//            cv::imshow("src_segment", src_segment);
//            cv::waitKey();

            drawResponse(templates, 2, display_comp_with_segment, cv::Point(matches[itr_match].x, matches[itr_match].y), detector->getT(0));
            //cv::imshow("display", display_comp_with_segment);


            std::vector<int> list_temppoints_on_segment;
            std::vector<int> list_whitepoints_on_segment;
            for(int it_mask=1; it_mask<mMat_v.size(); it_mask++)
            {
                // 1) count template point on the segment
                int temppoints_on_segment = countTemplatePointOnSegment(mMat_v[it_mask], templates, 2, cv::Point(matches[itr_match].x, matches[itr_match].y));
                list_temppoints_on_segment.push_back(temppoints_on_segment);

                // 2) count white pixels on segment
                int whitepoints_on_segment = countPixelsBetweenTwoValues(mMat_v[it_mask], 255, 255);
                list_whitepoints_on_segment.push_back(whitepoints_on_segment);
            }

            int max_ovlap = 0;
            int idx_max_ovlap = 0;
            for(int it=0; it<list_temppoints_on_segment.size(); it++)
            {

                if(max_ovlap < list_temppoints_on_segment[it])
                {
                    max_ovlap = list_temppoints_on_segment[it];
                    idx_max_ovlap = it;
                }
            }

//            std::cout << "ovlap_with_Other_Res : " << pixels_ovlap << std::endl;
//            std::cout << "ovlap_with_Segment   : " << max_ovlap << std::endl;
//            cv::waitKey();

            /// segments tono kaburiga ookiitokinomi jikkou
            if(max_ovlap > 126*0.95)
            {
                return matches;
            }
            else
            {
                matches.erase(matches.begin() + 0);
            }

        }
        else
        {
            matches.erase(matches.begin() + 0);
        }
    }

    return matches;
}

int calcDifDegree(int src, int temp)
{
    int dif = 0;

    if(90 < temp-src)
        src += 180;
    else if(temp - src < -90)
        temp +=180;

    dif = temp - src;
    if(dif < 0)
        dif *= -1;

    return dif;
}

/** ******* **/
/** SEGMENT **/
/** ******* **/

void robot_vision_recognition2::recognizedSegmentCallback(const T2_robot_vision::RecognizedSegmentConstPtr &notice)
{
    int jn = notice->recog_target.job_no;
    //inputimg_counter++;

    ROS_INFO_STREAM("job_no:" << jn << " T2_robot_vision:recognition2 seg->linemod nodelet start");
    log4cxx::LoggerPtr logptr = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);

    std::string src_root_path = ros::package::getPath("T2_robot_vision");

    //get config from recog2prm.txt
    Recog2Prm recog2_prm;
    std::string prm_file_path = src_root_path + "/data/recog2_prm.txt";
    std::string camera_name = "sr300";  //camera_name used because the parameters are named such as "sr300_...." in recog2_prm.txt

    ROS_INFO_STREAM("job_no:" << jn << " T2_robot_vision: recognition2 recog2_prm");
    Recog2PrmFileRead(prm_file_path, camera_name, recog2_prm);

    //if recog2_prm.mode_rotate_srcimg is 0, set step_roll = 360 to consider only roll = 0;
    if(recog2_prm.mode_rotate_srcimg == 0)
    {
        recog2_prm.step_roll = 360;
        ROS_INFO_STREAM("job_no:" << jn << " Recog2 => mode_rotate_srcimg:" << recog2_prm.mode_rotate_srcimg << " so set step_roll:" << recog2_prm.step_roll);
    }

    //avoid errors (step_roll == 0);
    if(recog2_prm.step_roll == 0)
    {
        recog2_prm.step_roll = 360;
        ROS_INFO_STREAM("job_no:" << jn << " Recog2 => step_roll:0 so set step_roll:" << recog2_prm.step_roll);
    }

    //SELECT Dummy Output Mode
    if (recog2_prm.mode_main_recognition2 == 0)
    {
        /**Dummy Output*/
        ROS_INFO_STREAM("job_no:" << jn << " T2_robot_vision:recognition2 Dummy Output Mode");

        T2_robot_vision::RecognizedItemPtr data(new T2_robot_vision::RecognizedItem);
        data->recog_target = notice->recog_target;
        data->calibrated_points.push_back(notice->calibrated_points[0]);

        if (notice->segmented_data.size() == 0)
        {
            ROS_INFO_STREAM("job_no:" << jn << " T2_robot_vision:recognition2 no segmented_data");
        }
        else
        {

            std::cout << "segmented_data.size() : " << notice->segmented_data.size() << std::endl;
            data->segmented_data = notice->segmented_data;

            T2_robot_vision::ItemizedData item;

            //dummy data:(default)Dex4_Orange_Tablet_RGB_5_45_1.bmp

            item.seg_id = 0;
            item.category = 27;
            item.yaw = 45;
            item.pitch = 5;
            item.roll = 0;
            item.score = 95;
            data->itemized_data.push_back(item);

            ROS_INFO_STREAM("job_no:" << jn << " category:" << item.category
                            << " pitch:" << item.pitch << " yaw:" << item.yaw << " roll:" << item.roll
                            << " score:" << item.score << " seg_id:" << item.seg_id);
        }
        data->rcg_module_index = 1;
        recognition_seg_pub_.publish(data);

        ROS_INFO_STREAM("job_no:" << jn << " T2_robot_vision:recognition2 nodelet end");
        return;
    }

    //SELECT Recognition Mode
    std::vector<cv_bridge::CvImagePtr> d_cvImg_v;
    std::vector<cv::Mat> cMat_v;
    std::vector<cv::Mat> dMat_v;

    T2_robot_vision::RecognizedItemPtr data(new T2_robot_vision::RecognizedItem);

    if (notice->segmented_data.size() == 0)
    {
        ROS_INFO_STREAM("job_no:" << jn << " T2_robot_vision:robot_vision_recognition2: no segmented_data");

        // output
        data->recog_target = notice->recog_target;
        data->calibrated_points.push_back(notice->calibrated_points[0]);
        data->rcg_module_index = 1;
        recognition_seg_pub_.publish(data);

        ofs_matchres << jn << ",-,no segment" << std::endl;

        ROS_INFO_STREAM("job_no:" << jn << " T2_robot_vision:recognition2 nodelet end");
        return;

    }
    else
    {

        if(recog2_prm.mode_size_inputimg)
        {

            const T2_robot_vision::RecognitionTarget recog_target = notice->recog_target;
            for (std::vector<T2_robot_vision::CapturedData>::const_iterator itr = recog_target.data.begin();
                 itr != recog_target.data.end();
                 itr++)
            {
                //2D_Data -> cv::Mat
                cv_bridge::CvImagePtr cv_color_ptr;
                cv_bridge::CvImagePtr cv_depth_ptr;
                try{
                    cv_color_ptr = cv_bridge::toCvCopy(itr->rgb_1, sensor_msgs::image_encodings::RGB8);
                    if (itr->depth.encoding == "16UC1")
                    {
                        sensor_msgs::Image depth;
                        depth.header = itr->depth.header;
                        depth.height = itr->depth.height;
                        depth.width = itr->depth.width;
                        depth.is_bigendian = itr->depth.is_bigendian;
                        depth.step = itr->depth.step;
                        depth.data = itr->depth.data;
                        depth.encoding = "mono16";
                        cv_depth_ptr = cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::MONO16);
                    }else{
                        cv_depth_ptr = cv_bridge::toCvCopy(itr->depth, sensor_msgs::image_encodings::MONO16);
                    }
                }
                catch (cv_bridge::Exception& e)
                {
                    ROS_ERROR("T2_robot_vision:cv_bridge exception:%s", e.what());
                }
                //Depth image : right and left are reversed
                cv::flip(cv_depth_ptr->image, cv_depth_ptr->image, 1);

                d_cvImg_v.push_back(cv_depth_ptr);
                cMat_v.push_back(cv_color_ptr->image);
                dMat_v.push_back(cv_depth_ptr->image);
                //            cv::imshow("color", cv_color_ptr->image);
                //            cv::imshow("depth", cv_depth_ptr->image * 255);
                //            cv::waitKey();
            }
        }

        if(recog2_prm.mode_size_inputimg == 0)
        {
            const T2_robot_vision::RecognitionTarget recog_target = notice->recog_target;
            for (std::vector<T2_robot_vision::CapturedData>::const_iterator itr = recog_target.data.begin();
                 itr != recog_target.data.end();
                 itr++)
            {
                //2D_Data -> cv::Mat
                cv_bridge::CvImagePtr cv_color_ptr;
                cv_bridge::CvImagePtr cv_depth_ptr;
                try
                {
                    cv_color_ptr = cv_bridge::toCvCopy(itr->rgb, sensor_msgs::image_encodings::RGB8);
                    if (itr->depth_1.encoding == "16UC1")
                    {
                        sensor_msgs::Image depth;
                        depth.header = itr->depth_1.header;
                        depth.height = itr->depth_1.height;
                        depth.width = itr->depth_1.width;
                        depth.is_bigendian = itr->depth_1.is_bigendian;
                        depth.step = itr->depth_1.step;
                        depth.data = itr->depth_1.data;
                        depth.encoding = "mono16";
                        cv_depth_ptr = cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::MONO16);
                    }else{
                        cv_depth_ptr = cv_bridge::toCvCopy(itr->depth_1, sensor_msgs::image_encodings::MONO16);
                    }
                }
                catch (cv_bridge::Exception& e)
                {
                    ROS_ERROR("T2_robot_vision:cv_bridge exception:%s", e.what());
                }
                d_cvImg_v.push_back(cv_depth_ptr);
                cMat_v.push_back(cv_color_ptr->image);
                dMat_v.push_back(cv_depth_ptr->image);
                //            cv::imshow("color", cv_color_ptr->image);
                //            cv::imshow("depth", cv_depth_ptr->image * 255);
                //            cv::waitKey();
            }
        }

        for(int i=0; i<cMat_v.size(); i++)
        {
            cvtColor(cMat_v[i], cMat_v[i], CV_RGB2BGR);
        }

        if(recog2_prm.mode_change_depth_range)
        {
            for(int j=0; j<dMat_v[0].rows; j++)
            {
                for(int i=0; i<dMat_v[0].cols; i++)
                {
                    dMat_v[0].at<unsigned short int>(j,i) = dMat_v[0].at<unsigned short int>(j,i)/10;
                }
            }
            //cv::imwrite(src_root_path + "/data/debug/debug_recog2_depth_ScalarChanged.png",dMat_v[0]);
        }

        cv::Mat depth16UC1;
        dMat_v[0].convertTo(depth16UC1, CV_16U);
        dMat_v[0] = depth16UC1;

        if(logptr->getLevel() == log4cxx::Level::getDebug())
        {
            cv::imwrite(src_root_path + "/data/debug/debug_recog2_color.jpg",cMat_v[0]);
            cv::imwrite(src_root_path + "/data/debug/debug_recog2_depth.jpg",dMat_v[0]);
        }

        // recognition process
        std::vector<uint32_t> seg_sx;
        std::vector<uint32_t> seg_ex;
        std::vector<uint32_t> seg_sy;
        std::vector<uint32_t> seg_ey;
        std::vector<uint32_t> seg_imgidx;
        std::vector<uint32_t> recog_category;
        std::vector<uint32_t> recog_yaw;
        std::vector<int32_t> recog_pitch;
        std::vector<uint32_t> recog_roll;
        std::vector<uint32_t> recog_score;
        std::vector<uint32_t> recog_seg_id;

        for (int itr = 0; itr < notice->segmented_data.size(); itr++)
        {
            seg_sx.push_back(notice->segmented_data[itr].sx);
            seg_ex.push_back(notice->segmented_data[itr].ex);
            seg_sy.push_back(notice->segmented_data[itr].sy);
            seg_ey.push_back(notice->segmented_data[itr].ey);
            seg_imgidx.push_back(notice->segmented_data[itr].img_idx);
        }

//        seg_ex.clear();
//        std::cout << __LINE__ << "seg_sx.size() : " << seg_sx.size() << std::endl;

        std::vector<cv::Mat> mMat_v;

        cv_bridge::CvImagePtr cv_mask_ptr;
        try
        {
            cv_mask_ptr = cv_bridge::toCvCopy(notice->segmented_data[0].mask, sensor_msgs::image_encodings::MONO8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("T2_robot_vision:cv_bridge exception:%s", e.what());
        }
        mMat_v.push_back(cv_mask_ptr->image);

        for(int itr=1; itr< notice->segmented_data.size();itr++)
        {
            cv::Mat mask = cv::Mat::ones(seg_ey[itr]-seg_sy[itr], seg_ex[itr]-seg_sx[itr], CV_8UC1);
            mMat_v.push_back(mask);

//            cv::imshow("mat_v[0]", mask*200);
//            cv::waitKey();
        }


        for(int i=0; i< notice->recog_target.category_id.size(); i++)
        {
            recog_category.push_back(notice->recog_target.category_id[i]);
        }
        //std::cout << "*******   recog_category:" << recog_category[0] << ", data->recog_target.category_id.size():" << notice->recog_target.category_id.size()  << std::endl;

        std::cout << "BeforeAlgMain mMat_v.size : " << mMat_v.size() << std::endl;

        T2_robot_vision::robot_vision_recognition2::SegLinemodAlgMain(cMat_v, dMat_v, seg_sx, seg_ex, seg_sy, seg_ey, seg_imgidx, mMat_v,
                                                                        recog_category, recog_yaw, recog_pitch, recog_roll, recog_score, recog_seg_id, recog2_prm.step_roll, jn,recog2_prm);//recog2_prm.step_roll);

        std::cout << "AfterAlgMain mMat_v.size : " << mMat_v.size() << std::endl;

        //check output data range
        if((recog_score.size()+1) != mMat_v.size())
        {
            ROS_ERROR_STREAM("Error : vectorsize of output datas is different from seg_imgid size");
        }

        //output
        data->recog_target = notice->recog_target;
        data->calibrated_points.push_back(notice->calibrated_points[0]);
        //data->segmented_data.push_back(notice->segmented_data[seg_id_symmax]);
        //data->segmented_data = notice->segmented_data;

        /** copy 1st segment **/
        data->segmented_data.push_back(notice->segmented_data[0]);

        for (int i = 0; i< recog_category.size(); i++)
        {
            SegmentedData seg_data;

            seg_data.sx = (int)((float)seg_sx[i+1] * (float)(1440.0/640.0) + 0.5) + 240;
            seg_data.ex = (int)((float)seg_ex[i+1] * (float)(1440.0/640.0) + 0.5) + 240;
            seg_data.sy = (int)((float)seg_sy[i+1] * (float)(1080.0/480.0) + 0.5);
            seg_data.ey = (int)((float)seg_ey[i+1] * (float)(1080.0/480.0) + 0.5);
            seg_data.img_idx = 0;
            seg_data.img_type = 1;
            seg_data.module_index = 4;

            std::cout <<"[" << i << "]vga rect -> HD rect translation" << std::endl;
            std::cout << "sx : " <<seg_sx[i+1] << " ->" << seg_data.sx << std::endl;
            std::cout << "ex : " <<seg_ex[i+1] << " ->" << seg_data.ex << std::endl;
            std::cout << "sy : " <<seg_sy[i+1] << " ->" << seg_data.sy << std::endl;
            std::cout << "ey : " <<seg_ey[i+1] << " ->" << seg_data.ey << std::endl;

            cv::Mat mMat_HD;
            int w = seg_data.ex - seg_data.sx;
            int h = seg_data.ey - seg_data.sy;

            cv::resize(mMat_v[i+1], mMat_HD, cv::Size(w, h), cv::INTER_LINEAR);

            sensor_msgs::ImagePtr ros_image_ptr = cv_bridge::CvImage(std_msgs::Header(), "mono8", mMat_HD).toImageMsg();
            seg_data.mask = *ros_image_ptr;

            data->segmented_data.push_back(seg_data);

            seg_data.ex -= 1;
            seg_data.ey -= 1;

//            std::cout << mMat_HD.cols << " col :" << seg_data.ex - seg_data.sx << std::endl;
//            std::cout << mMat_HD.rows << " row :" << seg_data.ey - seg_data.sy << std::endl;

            T2_robot_vision::ItemizedData item;
            item.category = recog_category[i];
            item.yaw = recog_yaw[i];
            item.pitch = recog_pitch[i];
            item.roll = recog_roll[i];
            item.score = recog_score[i] - 5;
            item.seg_id = recog_seg_id[i];
            item.module_index = 1;

            data->itemized_data.push_back(item);


            ROS_INFO_STREAM("job_no:" << jn
                            << " seg_id:" << item.seg_id
                            << " category:" << item.category
                            << " score:" << item.score
                            << " pitch:" << item.pitch
                            << " yaw:" << item.yaw
                            << " roll:" << item.roll
                             );
        }
    }
    data->rcg_module_index = 1;
    recognition_seg_pub_.publish(data);
    recognition_seg_2plane_pub_.publish(data);

    ROS_INFO_STREAM("job_no:" << jn << " T2_robot_vision:recognition2 seg->linemod nodelet end");
    return;
}

void T2_robot_vision::robot_vision_recognition2::SegLinemodAlgMain(std::vector<cv::Mat>& cMat_v,
                                                                     std::vector<cv::Mat>& dMat_v,
                                                                     std::vector<uint32_t>& seg_sx,
                                                                     std::vector<uint32_t>& seg_ex,
                                                                     std::vector<uint32_t>& seg_sy,
                                                                     std::vector<uint32_t>& seg_ey,
                                                                     std::vector<uint32_t>& seg_imgidx,
                                                                     std::vector<cv::Mat>& mMat_v,
                                                                     std::vector<uint32_t>& recog_category,
                                                                     std::vector<uint32_t>& recog_yaw,
                                                                     std::vector<int32_t>& recog_pitch,
                                                                     std::vector<uint32_t>& recog_roll,
                                                                     std::vector<uint32_t>& recog_score,
                                                                     std::vector<uint32_t>& recog_seg_id,
                                                                     int step_roll,
                                                                     int jn,
                                                                     Recog2Prm recog2_prm
                                                                     )
{

    //std::cout << "*******   recog_category"  << recog_category[0]  << std::endl;


    ROS_INFO("Job_no:%d T2_robot_vision:recognition2 SegLinemodAlgMain start", jn);
    log4cxx::LoggerPtr logptr = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);

    // Various settings and flags

    int num_classes = 0;
    int matching_threshold = recog2_prm.threshold_matching_score;
    cv::Size resize_size(640, 480);

    cv::Mat color, depth;
    std::string src_root_path = ros::package::getPath("T2_robot_vision");

    // Timers
    Timer match_timer;

    // Initialize LINEMOD data structures
    //cv::Ptr<cv::linemod::Detector> detector;
    num_classes = detector->numClasses();
    int num_modalities = (int)detector->getModalities().size();
    //int num_modalities = 1;

    double focal_length = 1.0;

    //original temp.csv read point

    color = cMat_v[0].clone();
    depth = dMat_v[0].clone();

    if(depth.size()!=color.size())
    {
        std::cout << "error : color.size != depth.size -> resize depth" << std::endl;
        cv::resize(depth, depth, color.size(), cv::INTER_NEAREST);
    }

    cv::Mat color_resized = cv::Mat::zeros(resize_size.height, resize_size.width, CV_8UC3);
    cv::Mat depth_resized = cv::Mat::ones(resize_size.height, resize_size.width, CV_16UC1) * INVALID_DEPTH_VALUE;

#ifdef MODE_USE_VGA
    //Depth image : right and left are reversed
    cv::flip(depth, depth, 1);
#endif
#ifdef MODE_USE_HD_ROI

    cv::Point cp((int)color.cols*0.5, (int)color.rows*0.5);
    int width = (int)((double)color.rows*((double)resize_size.width/(double)resize_size.height));
    int height = color.rows;
    cv::Point tl(cp.x - width*0.5, 0);
    cv::Rect rect(tl.x, tl.y, width, height);//size_srcimg.width, size_srcimg.height);

    std::cout << "cp:" << cp << std::endl;
    std::cout << "tl:" << tl << std::endl;
    std::cout << "rect:" << rect << std::endl;

    cv::Mat roi_HDimg_color = cv::Mat(color, rect).clone();
    cv::resize(roi_HDimg_color, color_resized, color_resized.size(), cv::INTER_LINEAR);
    color = color_resized.clone();

    //cv::imwrite("color_HDROI.jpg", color);

    cv::Mat roi_HDimg_depth = cv::Mat(depth, rect).clone();
    cv::resize(roi_HDimg_depth, depth_resized, depth_resized.size(), cv::INTER_NEAREST);
    depth = depth_resized.clone();
#endif
#ifdef MODE_USE_HD_RESIZE
    cv::resize(color, color_resized, color_resized.size(), cv::INTER_LINEAR);
    color = color_resized.clone();
    cv::resize(depth, depth_resized, depth_resized.size(), cv::INTER_NEAREST);
    depth = depth_resized.clone();
#endif

    //output best display image
    cv::Mat MatchingResultImage = cv::Mat::zeros(resize_size.height, resize_size.width, CV_8UC3);
    double max_similarity = 0;

    cv::Mat result_forsave_img;

//#if MODE_MATCH_ALLAREA

    color = color_resized;
    depth = depth_resized;

//    imshow("color", color);
//    cv::waitKey();

    if (color.empty())
    {
        std::cout << "can't read color" << std::endl;
    }
    if (depth.empty())
    {
        std::cout << "can't read depth" << std::endl;
    }

    /** resize mask image HD -> VGA **/
    std::vector<cv::Mat> masks;

    for (int itr_mask = 0; itr_mask<mMat_v.size(); itr_mask++)
    {
        // マスク処理
        cv::Mat mask_img = cv::Mat::zeros(resize_size.height, resize_size.width, CV_8UC1);

        int width = mMat_v[itr_mask].cols;
        int height = mMat_v[itr_mask].rows;
        int width_vga = mask_img.cols;

        int channels = mMat_v[itr_mask].channels();
        for (int j = 0; j<height; j++)
        {
            int step = j*width;
            int step_VGA = (j + seg_sy[itr_mask])*width_vga;
            for (int i = 0; i<width; i++)
            {
                int elm = i*mMat_v[itr_mask].elemSize();
                int elm_vga = (i + seg_sx[itr_mask])*mask_img.elemSize();
                for (int c = 0; c<channels; c++)
                {
                    if (mMat_v[itr_mask].data[step + elm + c] != 0)
                    {
                        //mMat_v[itr_mask].data[step + elm + c] = 255;
                        mask_img.data[step_VGA + elm_vga + c] = 255;
                    }
                }
            }
        }

        cv::dilate(mask_img, mask_img, cv::Mat(), cv::Point(-1, -1), 20);

        //perspective transform on mask
        cv::Point2f original[4];
        cv::Point2f translate[4];

#ifdef MODE_USE_HD_ROI
        original[0] = cv::Point2f(206, 150);
        original[1] = cv::Point2f(393, 149);
        original[2] = cv::Point2f(131, 413);
        original[3] = cv::Point2f(498, 404);

        translate[0] = cv::Point2f(201, 115);
        translate[1] = cv::Point2f(444, 110);
        translate[2] = cv::Point2f(118, 473);
        translate[3] = cv::Point2f(613, 461);
#endif

#ifdef MODE_USE_HD_RESIZE

        original[0] = cv::Point2f(185, 133);
        original[1] = cv::Point2f(349, 129);
        original[2] = cv::Point2f(122, 368);
        original[3] = cv::Point2f(451, 363);

        translate[0] = cv::Point2f(209, 109);
        translate[1] = cv::Point2f(374, 99);
        translate[2] = cv::Point2f(152, 417);
        translate[3] = cv::Point2f(482, 413);
#endif
        //check segarea
        cv::Mat perspectiveMatrix;
        perspectiveMatrix = cv::getPerspectiveTransform(original, translate);

        cv::Mat segarea_vga = (cv::Mat_<double>(3,4) <<
                               seg_sx[itr_mask],seg_ex[itr_mask],seg_sx[itr_mask],seg_ex[itr_mask],
                               seg_sy[itr_mask],seg_sy[itr_mask],seg_ey[itr_mask],seg_ey[itr_mask],
                               1,1,1,1);
        cv::Mat segarea_HD = (cv::Mat_<double>(3,4) << 0,0,0,0,0,0,0,0,0,0,0,0);

        segarea_HD = perspectiveMatrix * segarea_vga;

        ROS_DEBUG_STREAM("tlx: " << segarea_HD.at<double>(0,0) << ", tly: " << segarea_HD.at<double>(1,0));
        ROS_DEBUG_STREAM("trx: " << segarea_HD.at<double>(0,1) << ", try: " << segarea_HD.at<double>(1,1));
        ROS_DEBUG_STREAM("blx: " << segarea_HD.at<double>(0,2) << ", bly: " << segarea_HD.at<double>(1,2));
        ROS_DEBUG_STREAM("brx: " << segarea_HD.at<double>(0,3) << ", bry: " << segarea_HD.at<double>(1,3));

        int flag_available_segarea = 1;

        //about x
        if(segarea_HD.at<double>(0,0) >= resize_size.width)
            flag_available_segarea = 0;
        if(segarea_HD.at<double>(0,2) >= resize_size.width)
            flag_available_segarea = 0;
        if(segarea_HD.at<double>(0,1) <= 0)
            flag_available_segarea = 0;
        if(segarea_HD.at<double>(0,3) <= 0)
            flag_available_segarea = 0;
        //about y
        if(segarea_HD.at<double>(1,0) >= resize_size.height)
            flag_available_segarea = 0;
        if(segarea_HD.at<double>(1,1) >= resize_size.height)
            flag_available_segarea = 0;
        if(segarea_HD.at<double>(1,2) <= 0)
            flag_available_segarea = 0;
        if(segarea_HD.at<double>(1,3) <= 0)
            flag_available_segarea = 0;

        ROS_DEBUG_STREAM("flag_good_segarea : " << flag_available_segarea);


        cv::Mat mask_img2 = cv::Mat::zeros(resize_size.height, resize_size.width, CV_8UC1);

        if(flag_available_segarea)
        {
            cv::warpPerspective(mask_img, mask_img2, perspectiveMatrix, mask_img.size(),
                                cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0));
        }
        else
        {
            ROS_INFO("job_no:%d, recog2 => mask:%d, (segarea halt)", jn, itr_mask);
        }

        mask_img = mask_img2.clone();
        masks.push_back(mask_img);
    }

    std::vector<cv::Mat> sources;  //取得画像->Mat  [0]RGB, [1]Depth

    sources.push_back(color);
    sources.push_back(depth);

    cv::Mat display = sources[0].clone();

    std::vector<cv::linemod::Match> matches;
    std::vector<std::string> class_ids;
    std::vector<cv::Mat> quantized_images;

    detector->match(sources, (float)70.0, matches, class_ids, quantized_images);

    ROS_INFO_STREAM( "all matches : " << matches.size() );

    /** cut by strong thresholds**/

    bool flag_add_strong[200];
    //bool flag_add_weak[200] = {false};

    for(int i=0; i<200; i++)
    {
        flag_add_strong[i] = true;
    }

    for(int i=0; i<recog_category.size(); i++)
    {
        flag_add_strong[recog_category[i]] = false;

    }
    recog_category.clear();

    std::vector< cv::linemod::Match > cand_matches_weak;
    std::vector< cv::linemod::Match > cand_matches_strong;
    for(int itr_cand=0; itr_cand < matches.size() ; itr_cand++)
    {
        int idx_matched_class = atoi((matches[itr_cand].class_id).substr(5).c_str());
        int cat_id = TemplatesData[idx_matched_class].object;

        if( (flag_add_strong[cat_id] == false) && (matches[itr_cand].similarity > th_strong[cat_id]))
        {
            const std::vector<cv::linemod::Template>& templates = detector->getTemplates(matches[itr_cand].class_id, matches[itr_cand].template_id);

            cv::Mat cand_img_each = sources[0].clone();

            int flag_use = checkNormBriefness(templates, 2, cand_img_each, cv::Point(matches[itr_cand].x, matches[itr_cand].y), detector->getT(0));

            if(flag_use != 0)
            {
                flag_use = checkDaenScore(templates, 2, cand_img_each, cv::Point(matches[itr_cand].x, matches[itr_cand].y), detector->getT(0));
            }

            if(flag_use != 0)
            {
                cand_matches_strong.push_back(matches[itr_cand]);
                flag_add_strong[cat_id] = true;

                std::cout << "[" << itr_cand << "]:" << matches[itr_cand].class_id << ", " << matches[itr_cand].similarity << "% , cat:" << objlist[TemplatesData[idx_matched_class].object] << ", size:" << TemplatesData[idx_matched_class].power << std::endl;
                std::cout << "num_same_color : " << num_same_color <<  std::endl;

                // save OutputImage(2)
                drawResponse(templates, 2, cand_img_each, cv::Point(matches[itr_cand].x, matches[itr_cand].y), detector->getT(0));
            }
        }
    }

    std::vector<MatchResult> list_strong_output;

    for(int itr_cand=0; itr_cand < cand_matches_strong.size() ; itr_cand++)
    {
        std::cout << cand_matches_strong.size() << " : cand_matches_strong : " << std::endl;

        //select champion from matches
        MatchResult match_res;

        cv::linemod::Match match_champion = cand_matches_strong[itr_cand];

        match_res.mask = sources[0].clone();
        const std::vector<cv::linemod::Template>& templates = detector->getTemplates(match_champion.class_id, match_champion.template_id);
        createMaskFromFeaturePoints(templates, 2, match_res.mask, cv::Point(match_champion.x, match_champion.y), detector->getT(0));

        match_res.tlx = match_champion.x;
        match_res.tly = match_champion.y;
        match_res.brx = match_champion.x + match_res.mask.cols - 1;
        match_res.bry = match_champion.y + match_res.mask.rows - 1;

        match_res.maskid = itr_cand + 1;
        match_res.class_id = match_champion.class_id;
        match_res.similarity = match_champion.similarity;

        int idx_matched_class = atoi((match_champion.class_id).substr(5).c_str());
        match_res.int_class_id = idx_matched_class;
        match_res.temp = TemplatesData[idx_matched_class];

        list_strong_output.push_back(match_res);
    }

    //ROS_INFO_STREAM( "*** compare matches with segment areas *** " );
    /** comp with segment area **/
    //list_strong_output

    while(cand_matches_weak.size() > 0)
    {   
        cand_matches_weak = checkOverlapWithOtherResult(cand_matches_weak, list_strong_output, masks, TemplatesData);

        if(cand_matches_weak.size() == 0)
        {
            break;
        }

        //select champion from matches
        MatchResult match_res;

        cv::linemod::Match match_champion = cand_matches_weak[0];

        int idx_matched_class = atoi((match_champion.class_id).substr(5).c_str());
        match_res.int_class_id = idx_matched_class;
        match_res.temp = TemplatesData[idx_matched_class];

        /** is there the same object ??? **/
        if(flag_add_strong[match_res.temp.object] == true)
        {
            cand_matches_weak.erase(cand_matches_weak.begin());
            continue;
        }
        else
        {
            flag_add_strong[match_res.temp.object] = true;
        }

        match_res.mask = sources[0].clone();
        const std::vector<cv::linemod::Template>& templates = detector->getTemplates(match_champion.class_id, match_champion.template_id);
        createMaskFromFeaturePoints(templates, 2, match_res.mask, cv::Point(match_champion.x, match_champion.y), detector->getT(0));

        match_res.tlx = match_champion.x;
        match_res.tly = match_champion.y;
        match_res.brx = match_champion.x + match_res.mask.cols - 1;
        match_res.bry = match_champion.y + match_res.mask.rows - 1;

        match_res.maskid = list_strong_output.size();
        match_res.class_id = match_champion.class_id;
        match_res.similarity = match_champion.similarity;

        list_strong_output.push_back(match_res);
    }

    /** Set & Output recog datas **/

    // recognition process
    int seg0_sx = seg_sx[0];
    int seg0_ex = seg_ex[0];
    int seg0_sy = seg_sy[0];
    int seg0_ey = seg_ey[0];
    int seg0_imgidx = seg_imgidx[0];
    cv::Mat mMat0 = mMat_v[0];

    seg_sx.clear();
    seg_ex.clear();
    seg_sy.clear();
    seg_ey.clear();
    seg_imgidx.clear();
    mMat_v.clear();

    seg_sx.push_back(seg0_sx);
    seg_ex.push_back(seg0_ex);
    seg_sy.push_back(seg0_sy);
    seg_ey.push_back(seg0_ey);
    seg_imgidx.push_back(seg0_imgidx);
    mMat_v.push_back(mMat0);

//    std::cout << "seg_sx.size() : " << seg_sx.size() << std::endl;

    /** output vectors **/
    for (int itr_res = 0; itr_res<list_strong_output.size(); itr_res++)
    {
        //ROS_INFO("job_no:%d, recog2 => mask:%d, No matches found...", jn, itr_mask);
        recog_category.push_back(list_strong_output[itr_res].temp.object);
        recog_yaw.push_back(list_strong_output[itr_res].temp.yaw);
        recog_pitch.push_back(list_strong_output[itr_res].temp.pitch);
        recog_roll.push_back(list_strong_output[itr_res].temp.roll);
        recog_score.push_back(list_strong_output[itr_res].similarity);
        recog_seg_id.push_back(list_strong_output[itr_res].maskid);
        mMat_v.push_back(list_strong_output[itr_res].mask);

        seg_sx.push_back(list_strong_output[itr_res].tlx);
        seg_ex.push_back(list_strong_output[itr_res].brx);
        seg_sy.push_back(list_strong_output[itr_res].tly);
        seg_ey.push_back(list_strong_output[itr_res].bry);
        seg_imgidx.push_back(seg_imgidx.size());
    }

    ROS_INFO_STREAM( "job_no:" << jn << " T2_robot_vision:recognition2 SegLinemodAlgMain end");
}




/** **** **/
/** YOLO **/
/** **** **/



void robot_vision_recognition2::resultYOLOv2Callback(const T2_robot_vision::RecognizedItemConstPtr &notice)
{

    int jn = notice->recog_target.job_no;
    //inputimg_counter++;

    ROS_INFO_STREAM("job_no:" << jn << " T2_robot_vision:recognition2 nodelet start");
    log4cxx::LoggerPtr logptr = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);

    std::string src_root_path = ros::package::getPath("T2_robot_vision");

    //get config from recog2prm.txt
    Recog2Prm recog2_prm;
    std::string prm_file_path = src_root_path + "/data/recog2_prm.txt";
    std::string camera_name = "sr300";

    ROS_INFO_STREAM("job_no:" << jn << " T2_robot_vision: recognition2 recog2_prm");
    Recog2PrmFileRead(prm_file_path, camera_name, recog2_prm);

    //if recog2_prm.mode_rotate_srcimg is 0, set step_roll = 360 to consider only roll = 0;
    if(recog2_prm.mode_rotate_srcimg == 0)
    {
        recog2_prm.step_roll = 360;
        ROS_INFO_STREAM("job_no:" << jn << " Recog2 => mode_rotate_srcimg:" << recog2_prm.mode_rotate_srcimg << " so set step_roll:" << recog2_prm.step_roll);
    }

    //avoid errors (step_roll == 0);
    if(recog2_prm.step_roll == 0)
    {
        recog2_prm.step_roll = 360;
        ROS_INFO_STREAM("job_no:" << jn << " Recog2 => step_roll:0 so set step_roll:" << recog2_prm.step_roll);
    }


    T2_robot_vision::RecognizedItemPtr data(new T2_robot_vision::RecognizedItem);

    //SELECT Dummy Output Mode
    if (recog2_prm.mode_main_recognition2 == 0)
    {
        /**Dummy Output*/
        ROS_INFO_STREAM("job_no:" << jn << " T2_robot_vision:recognition2 Dummy Output Mode");

        //output
        data->recog_target = notice->recog_target;
        data->calibrated_points.push_back(notice->calibrated_points[0]);
        //data->segmented_data.push_back(notice->segmented_data[seg_id_symmax]);
        data->segmented_data = notice->segmented_data;

        if (notice->segmented_data.size() == 0)
        {
            ROS_INFO_STREAM("job_no:" << jn << " T2_robot_vision:recognition2 no segmented_data");
        }
        else
        {

            std::cout << "segmented_data.size() : " << notice->segmented_data.size() << std::endl;
            data->segmented_data = notice->segmented_data;

            T2_robot_vision::ItemizedData item;

            //dummy data:(default)Dex4_Orange_Tablet_RGB_5_45_1.bmp

            item.seg_id = 0;
            item.category = 27;
            item.yaw = 23;
            item.pitch = 56;
            item.roll = 78;
            item.score = 90;
            data->itemized_data.push_back(item);

            ROS_INFO_STREAM("job_no:" << jn << " category:" << item.category
                            << " pitch:" << item.pitch << " yaw:" << item.yaw << " roll:" << item.roll
                            << " score:" << item.score << " seg_id:" << item.seg_id);

            data->itemized_data.push_back(item);
        }
        data->rcg_module_index = 4;
        recognition_yolo_pub_.publish(data);

        ROS_INFO_STREAM("job_no:" << jn << " T2_robot_vision:recognition2 nodelet end");
        return;
    }

    //SELECT Recognition Mode
    std::vector<cv_bridge::CvImagePtr> d_cvImg_v;
    std::vector<cv::Mat> cMat_v;
    std::vector<cv::Mat> dMat_v;

    if (notice->segmented_data.size() == 0)
    {
        ROS_INFO_STREAM("job_no:" << jn << " T2_robot_vision:robot_vision_recognition2: no segmented_data");

        // output
        data->recog_target = notice->recog_target;
        data->calibrated_points.push_back(notice->calibrated_points[0]);
        data->rcg_module_index = 4;
        recognition_yolo_pub_.publish(data);

        ofs_matchres << jn << ",-,no segment" << std::endl;

        ROS_INFO_STREAM("job_no:" << jn << " T2_robot_vision:recognition2 nodelet end");
        return;
    }
    else
    {
        if(recog2_prm.mode_size_inputimg)
        {

            const T2_robot_vision::RecognitionTarget recog_target = notice->recog_target;
            for (std::vector<T2_robot_vision::CapturedData>::const_iterator itr = recog_target.data.begin();
                 itr != recog_target.data.end();
                 itr++)
            {
                //2D_Data -> cv::Mat
                cv_bridge::CvImagePtr cv_color_ptr;
                cv_bridge::CvImagePtr cv_depth_ptr;
                try{
                    cv_color_ptr = cv_bridge::toCvCopy(itr->rgb_1, sensor_msgs::image_encodings::RGB8);
                    if (itr->depth.encoding == "16UC1")
                    {
                        sensor_msgs::Image depth;
                        depth.header = itr->depth.header;
                        depth.height = itr->depth.height;
                        depth.width = itr->depth.width;
                        depth.is_bigendian = itr->depth.is_bigendian;
                        depth.step = itr->depth.step;
                        depth.data = itr->depth.data;
                        depth.encoding = "mono16";
                        cv_depth_ptr = cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::MONO16);
                    }else{
                        cv_depth_ptr = cv_bridge::toCvCopy(itr->depth, sensor_msgs::image_encodings::MONO16);
                    }
                }
                catch (cv_bridge::Exception& e)
                {
                    ROS_ERROR("T2_robot_vision:cv_bridge exception:%s", e.what());
                }
                //Depth image : right and left are reversed
                cv::flip(cv_depth_ptr->image, cv_depth_ptr->image, 1);

                d_cvImg_v.push_back(cv_depth_ptr);
                cMat_v.push_back(cv_color_ptr->image);
                dMat_v.push_back(cv_depth_ptr->image);
                cv::imshow("color", cv_color_ptr->image);
                cv::imshow("depth", cv_depth_ptr->image * 255);
                cv::waitKey();
            }
        }

        if(recog2_prm.mode_size_inputimg == 0)
        {
            const T2_robot_vision::RecognitionTarget recog_target = notice->recog_target;
            for (std::vector<T2_robot_vision::CapturedData>::const_iterator itr = recog_target.data.begin();
                 itr != recog_target.data.end();
                 itr++)
            {
                //2D_Data -> cv::Mat
                cv_bridge::CvImagePtr cv_color_ptr;
                cv_bridge::CvImagePtr cv_depth_ptr;
                try{
                    cv_color_ptr = cv_bridge::toCvCopy(itr->rgb, sensor_msgs::image_encodings::RGB8);
                    if (itr->depth_1.encoding == "16UC1")
                    {
                        sensor_msgs::Image depth;
                        depth.header = itr->depth_1.header;
                        depth.height = itr->depth_1.height;
                        depth.width = itr->depth_1.width;
                        depth.is_bigendian = itr->depth_1.is_bigendian;
                        depth.step = itr->depth_1.step;
                        depth.data = itr->depth_1.data;
                        depth.encoding = "mono16";
                        cv_depth_ptr = cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::MONO16);
                    }else{
                        cv_depth_ptr = cv_bridge::toCvCopy(itr->depth_1, sensor_msgs::image_encodings::MONO16);
                    }
                }
                catch (cv_bridge::Exception& e)
                {
                    ROS_ERROR("T2_robot_vision:cv_bridge exception:%s", e.what());
                }
                d_cvImg_v.push_back(cv_depth_ptr);
                cMat_v.push_back(cv_color_ptr->image);
                dMat_v.push_back(cv_depth_ptr->image);
//                cv::imshow("color", cv_color_ptr->image);
//                cv::imshow("depth", cv_depth_ptr->image * 255);
//                cv::waitKey();
            }
        }

        for(int i=0; i<cMat_v.size(); i++)
        {
            cvtColor(cMat_v[i], cMat_v[i], CV_RGB2BGR);
        }

        if(recog2_prm.mode_change_depth_range)
        {
            for(int j=0; j<dMat_v[0].rows; j++)
            {
                for(int i=0; i<dMat_v[0].cols; i++)
                {
                    dMat_v[0].at<unsigned short int>(j,i) = dMat_v[0].at<unsigned short int>(j,i)/10;
                }
            }
            //cv::imwrite(src_root_path + "/data/debug/debug_recog2_depth_ScalarChanged.png",dMat_v[0]);
        }

        //printf("c:%d, d:%d -> ",cMat_v[0].type(),dMat_v[0].type());

        cv::Mat depth16UC1;
        dMat_v[0].convertTo(depth16UC1, CV_16U);
        dMat_v[0] = depth16UC1;

        if(logptr->getLevel() == log4cxx::Level::getDebug())
        {
            cv::imwrite(src_root_path + "/data/debug/debug_recog2_color.jpg",cMat_v[0]);
            cv::imwrite(src_root_path + "/data/debug/debug_recog2_depth.jpg",dMat_v[0]);
        }

        // recognition process
        std::vector<uint32_t> seg_sx;
        std::vector<uint32_t> seg_ex;
        std::vector<uint32_t> seg_sy;
        std::vector<uint32_t> seg_ey;
        std::vector<uint32_t> seg_imgidx;
        std::vector<uint32_t> recog_category;
        std::vector<uint32_t> recog_yaw;
        std::vector<int32_t> recog_pitch;
        std::vector<uint32_t> recog_roll;
        std::vector<uint32_t> recog_score;
        std::vector<uint32_t> recog_seg_id;

        for (int itr = 0; itr < notice->segmented_data.size(); itr++)
        {
            seg_sx.push_back(notice->segmented_data[itr].sx);
            seg_ex.push_back(notice->segmented_data[itr].ex);
            seg_sy.push_back(notice->segmented_data[itr].sy);
            seg_ey.push_back(notice->segmented_data[itr].ey);
            seg_imgidx.push_back(notice->segmented_data[itr].img_idx);
        }
        //ROS_INFO("seg_sx %d\n", seg_sx[0]);


        std::vector<cv::Mat> mMat_v;

        int idx_cMat_v = 0;

        for (int itr = 0; itr < notice->segmented_data.size(); itr++)
        {
            seg_sx.push_back(notice->segmented_data[itr].sx);
            seg_ex.push_back(notice->segmented_data[itr].ex);
            seg_sy.push_back(notice->segmented_data[itr].sy);
            seg_ey.push_back(notice->segmented_data[itr].ey);
            seg_imgidx.push_back(notice->segmented_data[itr].img_idx);
            recog_category.push_back(notice->itemized_data[itr].category);
            //recog_category.push_back(51);
            //std::cout << notice->itemized_data[itr].category << std::endl;
        }

        for (int itr = 0; itr < notice->segmented_data.size(); itr++)
        {
            cv::Mat mask_yolo = cv::Mat::zeros(cMat_v[idx_cMat_v].rows, cMat_v[idx_cMat_v].cols, CV_8UC1);

            cv::Rect rect_segarea(seg_sx[itr], seg_sy[itr], seg_ex[itr] - seg_sx[itr], seg_ey[itr] - seg_sy[itr]);
            cv::rectangle(mask_yolo, rect_segarea, 255, CV_FILLED);

            mMat_v.push_back(mask_yolo);
        }

        //std::cout << "BeforeAlgMain mMat_v.size : " << mMat_v.size() << std::endl;

//        T2_robot_vision::robot_vision_recognition2::RecognizeAlgMain(cMat_v, dMat_v, seg_sx, seg_ex, seg_sy, seg_ey, seg_imgidx, mMat_v,
//                                                                        recog_category, recog_yaw, recog_pitch, recog_roll, recog_score, recog_seg_id, recog2_prm.step_roll, jn,recog2_prm);//recog2_prm.step_roll);


        T2_robot_vision::robot_vision_recognition2::YoloLinemodAlgMain(cMat_v, dMat_v, seg_sx, seg_ex, seg_sy, seg_ey, seg_imgidx, mMat_v,
                                                                        recog_category, recog_yaw, recog_pitch, recog_roll, recog_score, recog_seg_id, recog2_prm.step_roll, jn,recog2_prm);//recog2_prm.step_roll);


        //std::cout << "AfterAlgMain mMat_v.size : " << mMat_v.size() << std::endl;

//        for(int i=0;i<recog_seg_id.size();i++)
//        {
//            std::cout << "AfterAlgMain recog_category" << i << " : " << recog_category[i] << std::endl;
//            std::cout << "AfterAlgMain recog_yaw" << i << " : " << recog_yaw[i] << std::endl;
//            std::cout << "AfterAlgMain recog_pitch" << i << " : " << recog_pitch[i] << std::endl;
//            std::cout << "AfterAlgMain recog_roll" << i << " : " << recog_roll[i] << std::endl;
//            std::cout << "AfterAlgMain recog_score" << i << " : " << recog_score[i] << std::endl;
//            std::cout << "AfterAlgMain recog_seg_id" << i << " : " << recog_seg_id[i] << std::endl;
//        }

        //check output data range
        if(recog_score.size() != mMat_v.size())
        {
            ROS_ERROR_STREAM("Error : vectorsize of output datas is different from seg_imgid size");
        }

        //output
        data->recog_target = notice->recog_target;
        data->calibrated_points.push_back(notice->calibrated_points[0]);
        //data->segmented_data.push_back(notice->segmented_data[seg_id_symmax]);
        //data->segmented_data = notice->segmented_data;

        for (int i = 0; i<mMat_v.size(); i++)
        {

            SegmentedData seg_data;

            seg_data.sx = (int)((float)seg_sx[i] * (float)(1440.0/640.0) + 0.5) + 240;
            seg_data.ex = (int)((float)seg_ex[i] * (float)(1440.0/640.0) + 0.5) + 240;
            seg_data.sy = (int)((float)seg_sy[i] * (float)(1080.0/480.0) + 0.5);
            seg_data.ey = (int)((float)seg_ey[i] * (float)(1080.0/480.0) + 0.5);
            seg_data.img_idx = 0;
            seg_data.img_type = 1;
            seg_data.module_index = 4;

//            std::cout <<"[" << i << "]vga rect -> HD rect translation" << std::endl;
//            std::cout << "sx : " <<seg_sx[i] << " ->" << seg_data.sx << std::endl;
//            std::cout << "ex : " <<seg_ex[i] << " ->" << seg_data.ex << std::endl;
//            std::cout << "sy : " <<seg_sy[i] << " ->" << seg_data.sy << std::endl;
//            std::cout << "ey : " <<seg_ey[i] << " ->" << seg_data.ey << std::endl;

            cv::Mat mMat_HD;
            int w = seg_data.ex - seg_data.sx;
            int h = seg_data.ey - seg_data.sy;

            cv::resize(mMat_v[i], mMat_HD, cv::Size(w, h), cv::INTER_LINEAR);

            sensor_msgs::ImagePtr ros_image_ptr = cv_bridge::CvImage(std_msgs::Header(), "mono8", mMat_HD).toImageMsg();
            seg_data.mask = *ros_image_ptr;

            data->segmented_data.push_back(seg_data);

            ///???? before push_back
            seg_data.ex -= 1;
            seg_data.ey -= 1;

            T2_robot_vision::ItemizedData item;
            item.category = recog_category[i];
            item.yaw = recog_yaw[i];
            item.pitch = recog_pitch[i];
            item.roll = recog_roll[i];
            item.score = recog_score[i];
            item.seg_id = recog_seg_id[i];
            item.module_index = 4;

            ROS_INFO_STREAM("job_no:" << jn
                            << " seg_id:" << item.seg_id
                            << " category:" << item.category
                            << " score:" << item.score
                            << " pitch:" << item.pitch
                            << " yaw:" << item.yaw
                            << " roll:" << item.roll
                             );


            if( item.score >= recog2_prm.threshold_matching_score )
            {

                data->itemized_data.push_back(item);

                ofs_matchres << jn << ","
                             //<< inputimg_counter << ","
                             << item.seg_id << ","
                             << item.score << ","
                             << item.category << ","
                             << item.yaw << ","
                             << item.pitch << ","
                             << item.roll << "," ;//<< std::endl;
            }
            else if( item.score == SCORE_INVALID_SEGMENT )
            {
//                ofs_matchres << jn << ","
//                             << inputimg_counter << ","
//                             << recog_seg_id[i] << ",invalid segment" << std::endl;
            }
            else
            {

//                ofs_matchres << jn << ","
//                             << inputimg_counter << ","
//                             << item.seg_id << ",no match result" << std::endl;
            }
        }

        ofs_matchres << std::endl;

//        for(int i=0;i < data->itemized_data.size();i++)
//        {
//            std::cout << "data->itemized_data" << i << " : " << data->itemized_data[i].seg_id << std::endl;
//        }

    }

    data->rcg_module_index = 4;
    recognition_yolo_pub_.publish(data);

    ROS_INFO_STREAM("job_no:" << jn << " T2_robot_vision:recognition2 nodelet end");
    return;

}



void T2_robot_vision::robot_vision_recognition2::YoloLinemodAlgMain(std::vector<cv::Mat>& cMat_v,
                                                                     std::vector<cv::Mat>& dMat_v,
                                                                     std::vector<uint32_t>& seg_sx,
                                                                     std::vector<uint32_t>& seg_ex,
                                                                     std::vector<uint32_t>& seg_sy,
                                                                     std::vector<uint32_t>& seg_ey,
                                                                     std::vector<uint32_t>& seg_imgidx,
                                                                     std::vector<cv::Mat>& mMat_v,
                                                                     std::vector<uint32_t>& recog_category,
                                                                     std::vector<uint32_t>& recog_yaw,
                                                                     std::vector<int32_t>& recog_pitch,
                                                                     std::vector<uint32_t>& recog_roll,
                                                                     std::vector<uint32_t>& recog_score,
                                                                     std::vector<uint32_t>& recog_seg_id,
                                                                     int step_roll,
                                                                     int jn,
                                                                     Recog2Prm recog2_prm
                                                                     )
{

    ROS_INFO("Job_no:%d T2_robot_vision:recognition2 YoloLinemodAlgMain start", jn);
    log4cxx::LoggerPtr logptr = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);

    // Various settings and flags

    int num_classes = 0;
    int matching_threshold = recog2_prm.threshold_matching_score;
    cv::Size resize_size(640, 480);

    cv::Mat color, depth;
    std::string src_root_path = ros::package::getPath("T2_robot_vision");

    // Timers
    Timer match_timer;

    // Initialize LINEMOD data structures
    //cv::Ptr<cv::linemod::Detector> detector;
    num_classes = detector->numClasses();
    int num_modalities = (int)detector->getModalities().size();
    //int num_modalities = 1;

    double focal_length = 1.0;

    //original temp.csv read point

    color = cMat_v[0].clone();
    depth = dMat_v[0].clone();

	if(depth.size()!=color.size())
	{
		std::cout << "error : color.size != depth.size -> resize depth" << std::endl;
		cv::resize(depth, depth, color.size(), cv::INTER_NEAREST);
	}
    cv::Mat color_resized = cv::Mat::zeros(resize_size.height, resize_size.width, CV_8UC3);
    cv::Mat depth_resized = cv::Mat::ones(resize_size.height, resize_size.width, CV_16UC1) * INVALID_DEPTH_VALUE;

    //parameters for cut&resize roi to VGAsize from HDsize
    //cv::Point cp(960, 540);//(colorHD.cols*0.5, colorHD.rows*0.5);
    //cv::Point tl(cp.x - 720, 0);//(cp.x - size_srcimg.width*0.5, cp.y-size_srcimg.height*0.5);
    //cv::Rect rect(tl.x, tl.y, 1440, 1080);//size_srcimg.width, size_srcimg.height);

	cv::Point cp((int)color.cols*0.5, (int)color.rows*0.5);
	int width = (int)((double)color.rows*((double)resize_size.width/(double)resize_size.height));
    int height = color.rows;
    cv::Point tl(cp.x - width*0.5, 0);
    cv::Rect rect(tl.x, tl.y, width, height);//size_srcimg.width, size_srcimg.height);

	std::cout << "cp:" << cp << std::endl;
	std::cout << "tl:" << tl << std::endl;
	std::cout << "rect:" << rect << std::endl;
	
    cv::Mat roi_HDimg_color = cv::Mat(color, rect).clone();
    cv::resize(roi_HDimg_color, color_resized, color_resized.size(), cv::INTER_LINEAR);
    color = color_resized.clone();

    cv::Mat roi_HDimg_depth = cv::Mat(depth, rect).clone();
    cv::resize(roi_HDimg_depth, depth_resized, depth_resized.size(), cv::INTER_NEAREST);
    depth = depth_resized.clone();

    //output best display image
    cv::Mat MatchingResultImage = cv::Mat::zeros(resize_size.height, resize_size.width, CV_8UC3);
    double max_similarity = 0;

    cv::Mat result_forsave_img;


#if MODE_MATCH_ALLAREA

    color = color_resized;
    depth = depth_resized;

    if (color.empty())
    {
        std::cout << "can't read color" << std::endl;
    }
    if (depth.empty())
    {
        std::cout << "can't read depth" << std::endl;
    }

    /** resize mask image HD -> VGA **/
    std::vector<cv::Mat> masks;

    for (int itr_mask = 0; itr_mask<mMat_v.size(); itr_mask++)
    {        
        cv::Mat mask_resized = cv::Mat::zeros(resize_size.height, resize_size.width, CV_8UC1);

        cv::Mat roi_HDimg_mask = cv::Mat(mMat_v[itr_mask], rect).clone();
        cv::resize(roi_HDimg_mask, mask_resized, mask_resized.size(), cv::INTER_NEAREST);

        masks.push_back(mask_resized);

//        cv::imshow("mMat_v[itr_mask]",mMat_v[itr_mask]);
//        cv::imshow("mask_img",masks[itr_mask]);
//        cv::waitKey();
    }

//    std::cout << "mMat_v.size : " <<  mMat_v.size() << std::endl;
//    std::cout << "masks.size : " <<  masks.size() << std::endl;

    std::vector<cv::Mat> sources;  //取得画像->Mat  [0]RGB, [1]Depth

    sources.push_back(color);
    sources.push_back(depth);

    cv::Mat display = sources[0].clone();

    std::vector<cv::linemod::Match> matches;
    std::vector<std::string> class_ids;
    std::vector<cv::Mat> quantized_images;

    detector->match(sources, (float)matching_threshold, matches, class_ids, quantized_images);

    ROS_INFO_STREAM( "all matches : " << matches.size() );

    int classes_visited = 0;
    std::set<std::string> visited;

//    /** draw all candidates on src_img & save **/
//    for (int i = 0; (i < (int)matches.size()) && (classes_visited < num_classes); ++i)
//    {
//        cv::linemod::Match m = matches[i];

//        if (visited.insert(m.class_id).second)
//        {
//            ++classes_visited;

////            printf("Similarity: %5.1f%%; x: %3d; y: %3d; class: %s\n",
////                   m.similarity, m.x, m.y, m.class_id.c_str());

//            // Draw matching template
//            const std::vector<cv::linemod::Template>& templates = detector->getTemplates(m.class_id, m.template_id);
//            drawResponse(templates, num_modalities, display, cv::Point(m.x, m.y), detector->getT(0));
//        }

//        std::cout << m.similarity << std::endl;
//        cv::imshow("display", display);
//        cv::waitKey();
    //}

    //cv::imshow("display", display);
    cv::imwrite(src_root_path + "/data/debug/debug_recog2_resultimg_withTemplate/"
                + "jn" + int_to_string(jn)
                //+ "_img" + int_to_string(inputimg_counter)
                + "_AllCandidates"
                + ".png", display);

    std::vector<uint32_t> seg_sx_vga = seg_sx;
    std::vector<uint32_t> seg_sy_vga = seg_sy;
    std::vector<uint32_t> seg_ex_vga = seg_ex;
    std::vector<uint32_t> seg_ey_vga = seg_ey;
    std::vector<cv::Rect> seg_rect;

    int left = 240;
    for(int itr_mask = 0; itr_mask < mMat_v.size(); itr_mask++)
    {

        std::cout << "cat:" << recog_category[itr_mask] << std::endl;
        //cv::waitKey();

        //std::cout << seg_sx[itr_mask] - left << std::endl;
        if(seg_sx[itr_mask] > left)
        {
            seg_sx_vga[itr_mask] = (seg_sx[itr_mask] - left)/2.25;
        }
        else
        {
            seg_sx_vga[itr_mask] = 0;
        }
        if(seg_ex[itr_mask] > left)
        {
            seg_ex_vga[itr_mask] = (seg_ex[itr_mask] - left)/2.25;
        }
        else
        {
            seg_ex_vga[itr_mask] = 0;
        }
        seg_sy_vga[itr_mask] = seg_sy[itr_mask]/2.25;
        seg_ey_vga[itr_mask] = seg_ey[itr_mask]/2.25;
        cv::Rect rect_mask(seg_sx_vga[itr_mask],seg_sy_vga[itr_mask],seg_ex_vga[itr_mask]-seg_sx_vga[itr_mask],seg_ey_vga[itr_mask]-seg_sy_vga[itr_mask]);

        std::cout << seg_sx[itr_mask]  << "->" << seg_sx_vga[itr_mask] << std::endl;
        std::cout << seg_sy[itr_mask]  << "->" << seg_sy_vga[itr_mask] << std::endl;
        std::cout << seg_ex[itr_mask]  << "->" << seg_ex_vga[itr_mask] << std::endl;
        std::cout << seg_ey[itr_mask]  << "->" << seg_ey_vga[itr_mask] << std::endl;

        seg_rect.push_back(rect_mask);
        std::cout << rect_mask << std::endl;

        cv::Mat disp = cv::Mat::zeros(480,640,CV_8UC1);

        cv::rectangle(disp, rect_mask, 255, CV_FILLED);
        //cv::imshow("rect", disp);
        //cv::waitKey();
    }

    std::vector< cv::linemod::Match > result_matches;
    double TH_ovlap = 0.75;

    /** ref segarea from yolo **/
    for(int itr_mask = 0; itr_mask < mMat_v.size(); itr_mask++)
    {


        std::vector< cv::linemod::Match > matches_cutted;// = matches;
        std::vector< cv::linemod::Match > candidates = matches;

        cv::Rect rect_vga(0,0,680,480);
        cv::Rect rect_segarea_on_vga = rect_vga & seg_rect[itr_mask] ;

        if(rect_segarea_on_vga.width == seg_rect[itr_mask].width)
        {
            std::cout << "seg[" <<  itr_mask << "] : " <<"inside of vga" << std::endl;
            for(int itr_cand=0;itr_cand < candidates.size();itr_cand++)
            {
                int idx_matched_class = atoi((matches[itr_cand].class_id).substr(5).c_str());

                if(recog_category[itr_mask] == TemplatesData[idx_matched_class].object)
                {
                    //std::cout << TemplatesData[idx_matched_class].object << std::endl;

                    int classid_candidate = atoi((candidates[itr_cand].class_id).substr(5).c_str());
                    cv::Rect rect_candidate(candidates[itr_cand].x, candidates[itr_cand].y
                                            , TemplatesData[classid_candidate].width, TemplatesData[classid_candidate].height);

                    cv::Rect rect_overlap = seg_rect[itr_mask] & rect_candidate;
                    int size_rect_overlap = rect_overlap.width * rect_overlap.height;

                    //std::cout << size_rect_overlap << std::endl;

                    if(size_rect_overlap > (rect_candidate.width * rect_candidate.height) * TH_ovlap)
                    {

                        const std::vector<cv::linemod::Template>& templates_cand = detector->getTemplates(candidates[itr_cand].class_id, candidates[itr_cand].template_id);

                        cv::Mat cand_img_each = sources[0].clone();

                        int flag_use = checkDaenScore(templates_cand, 2, cand_img_each, cv::Point(candidates[itr_cand].x, candidates[itr_cand].y), detector->getT(0));

                        if(flag_use)
                        {
                            if(-30 < TemplatesData[classid_candidate].pitch && TemplatesData[classid_candidate].pitch < 30)
                                flag_use = 0;
                        }

                        if(flag_use)
                        {
                            matches_cutted.push_back(candidates[itr_cand]);
                        }
                    }
                }
            }
            //std::cout << itr_mask << ":" << matches_cutted.size() << std::endl;
        }
        else
        {
            std::cout << "seg[" <<  itr_mask << "] : " << "out of vga" << std::endl;
        }

        std::cout << "seg[" <<  itr_mask << "] : " << matches_cutted.size() << std::endl;

        if(matches_cutted.size()>0)
        {
            //flag_use = checkDaenScore(templates, 2, cand_img_each, cv::Point(matches[itr_cand].x, matches[itr_cand].y), detector->getT(0));

            result_matches.push_back(matches_cutted[0]);
            int idx_matched_class = atoi((result_matches[itr_mask].class_id).substr(5).c_str());

            cv::Mat result_img_all = sources[0].clone();
            //OutputImage(2) -> result_img_each : show 1 template on 1 image.
            cv::Mat result_img_each = sources[0].clone();
            const std::vector<cv::linemod::Template>& templates = detector->getTemplates(matches_cutted[0].class_id, matches_cutted[0].template_id);
            drawResponse(templates, 2, result_img_each, cv::Point(matches_cutted[0].x, matches_cutted[0].y), detector->getT(0));
            drawResponse(templates, 2, result_img_all, cv::Point(matches_cutted[0].x, matches_cutted[0].y), detector->getT(0));
            createMaskFromFeaturePoints(templates, 2, result_img_all, cv::Point(matches_cutted[0].x, matches_cutted[0].y), detector->getT(0));
            cv::Mat masked_result_img_each;
            //bitwise_and(result_img_each, masked_result_img_each, masks[itr_mask]);
            result_img_each.copyTo(masked_result_img_each, masks[itr_mask]);

            // save OutputImage(2)
            cv::imwrite(src_root_path + "/data/debug/debug_recog2_resultimg_withTemplate/"
                        + "jn" + int_to_string(jn)
                        //+ "_img" + int_to_string(inputimg_counter)
                        + "_yolo" + int_to_string(itr_mask)
                        + "_obj" + int_to_string(TemplatesData[idx_matched_class].object)
                        + "_sym" + int_to_string((int)matches_cutted[0].similarity)
                    + "_y" + int_to_string(TemplatesData[idx_matched_class].yaw)
                    + "_p" + int_to_string(TemplatesData[idx_matched_class].pitch)
                    + "_r" + int_to_string(TemplatesData[idx_matched_class].roll)
                    + ".png", masked_result_img_each);

            /** YOLOLINEMASK_OFF **/
                  cv::linemod::Match match_champion = matches_cutted[0];

                  MatchResult match_res;

                  match_res.mask = sources[0].clone();
                  //const std::vector<cv::linemod::Template>& templates = detector->getTemplates(match_champion.class_id, match_champion.template_id);
                  createMaskFromFeaturePoints(templates, 2, match_res.mask, cv::Point(match_champion.x, match_champion.y), detector->getT(0));

                  match_res.tlx = match_champion.x;
                  match_res.tly = match_champion.y;
                  match_res.brx = match_champion.x + match_res.mask.cols - 1;
                  match_res.bry = match_champion.y + match_res.mask.rows - 1;

                  match_res.maskid = itr_mask;
                  match_res.class_id = match_champion.class_id;
                  match_res.similarity = match_champion.similarity;

                  //int idx_matched_class = atoi((match_champion.class_id).substr(5).c_str());
                  match_res.int_class_id = idx_matched_class;
                  match_res.temp = TemplatesData[idx_matched_class];

                  seg_sx[itr_mask] = match_res.tlx;
                  seg_sy[itr_mask] = match_res.tly;
                  seg_ex[itr_mask] = match_res.brx;
                  seg_ey[itr_mask] = match_res.bry;

                  mMat_v[itr_mask] = match_res.mask.clone();

              }
              else
              {
                  cv::linemod::Match empty;
                  empty.class_id = "null";
                  result_matches.push_back(empty);

                  //cv::linemod::Match match_champion = cv::Mat::zeros(cv::Size(50,50), CV_8UC1);

                  MatchResult match_res;

                  match_res.mask = sources[0].clone();
                  match_res.tlx = 100;
                  match_res.tly = 100;
                  match_res.brx = 150;
                  match_res.bry = 150;
                  match_res.maskid = itr_mask;
                  match_res.class_id = empty.class_id;
                  match_res.similarity = 0;

                  seg_sx[itr_mask] = match_res.tlx;
                  seg_sy[itr_mask] = match_res.tly;
                  seg_ex[itr_mask] = match_res.brx;
                  seg_ey[itr_mask] = match_res.bry;

                  mMat_v[itr_mask] = cv::Mat::zeros(cv::Size(50,50), CV_8UC1);
              }
    }

    /** prepare output vectors **/
    for (int itr_mask = 0; itr_mask<mMat_v.size(); itr_mask++)
    {
        //ROS_INFO("job_no:%d, recog2 => mask:%d, No matches found...", jn, itr_mask);
        //recog_category.push_back(0);
        recog_yaw.push_back(0);
        recog_pitch.push_back(0);
        recog_roll.push_back(0);
        recog_score.push_back(0);
        recog_seg_id.push_back(itr_mask);
    }

    for (int itr_mask = 0; itr_mask<mMat_v.size(); itr_mask++)
    {

        if(result_matches[itr_mask].class_id != "null")
        {
            int idx_matched_class = atoi((result_matches[itr_mask].class_id).substr(5).c_str());

            recog_yaw[itr_mask] = TemplatesData[idx_matched_class].yaw;
            recog_pitch[itr_mask] = TemplatesData[idx_matched_class].pitch;
            recog_roll[itr_mask] = TemplatesData[idx_matched_class].roll;
            recog_score[itr_mask] = result_matches[itr_mask].similarity;

        }
        else
        {
            recog_yaw[itr_mask] = 0;
            recog_pitch[itr_mask] = 0;
            recog_roll[itr_mask] = 0;
            recog_score[itr_mask] = 0;
        }

//        std::cout << "recog_yaw[" << itr_mask << "] : " << recog_yaw[itr_mask] <<  std::endl;
//        std::cout << "recog_pitch[" << itr_mask << "] : " << recog_pitch[itr_mask] <<  std::endl;
//        std::cout << "recog_roll[" << itr_mask << "] : " << recog_roll[itr_mask] <<  std::endl;
//        std::cout << "recog_score[" << itr_mask << "] : " << recog_score[itr_mask] <<  std::endl;

//        std::cout << "seg_sx[" << itr_mask << "] : " << seg_sx[itr_mask] <<  std::endl;
//        std::cout << "seg_sy[" << itr_mask << "] : " << seg_sy[itr_mask] <<  std::endl;
//        std::cout << "seg_ex[" << itr_mask << "] : " << seg_ex[itr_mask] <<  std::endl;
//        std::cout << "seg_ey[" << itr_mask << "] : " << seg_ey[itr_mask] <<  std::endl;

//        cv::imshow("mMat" + itr_mask , mMat_v[itr_mask]);
//        cv::waitKey();
//        cv::destroyAllWindows();
    }


#endif

    ROS_INFO_STREAM( "job_no:" << jn << " T2_robot_vision:recognition2 YoloLinemodAlgMain end");
}

} // namespace T2_robot_vision

