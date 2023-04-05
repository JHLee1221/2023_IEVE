//#define ROS
#define CAMERA_SHOW
#define CAMERA_SHOW_MORE

#ifdef ROS
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sys/wait.h>
#include <terrio.h>
#include <unistd.h>
#include "std_msgs/int16.h"

#endif

#include <opencv4/opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>
#include <fcntl.h>
#include <time.h>

#define PI 3.1415926

using namespace std;
using namespace cv;

#define HIGH 1
#define LOW 0

const int Width = 320;
const int Height = 240;
const int XHalf = (Width/2);
const int YHalf = (Height/2);
const int YPoint[2] = { 20, YHalf - 20 };  //y point
const float slope_threshold = 0.5;
const Scalar Red = Scalar(0, 0, 255);
const Scalar Blue = Scalar(255, 0 ,0);
const Scalar Yellow = Scalar(50, 250, 250);
const Scalar Sky = Scalar(215, 200, 60);
const Scalar Pink = Scalar(220, 110, 230);

void show_lines(Mat &img, vector<Vec4i> &lines, Scalar color = Scalar(0, 0, 0), int thickness=2)
{
    bool color_gen = false;

    if (color == Scalar(0, 0, 0))
        color_gen = true;
    for (int i = 0; i < lines.size(); i++)
    {
        if (color_gen == true)
            color = Scalar(rand() % 256, rand() % 256, rand() % 256);
        line(img, Point(lines[i][0], lines[i][1]), Point(lines[i][2], lines[i][3]), color, thickness);
    }
}

void split_left_right(vector<Vec4i>lines, vector<Vec4i>&left_lines, vector<Vec4i>&right_lines)
{
    vector<float> slopes;
    vector<Vec4i> new_lines;

    for (int i =0; i< lines.size(); i++)
    {
        int x1 = lines[i][0];
        int y1 = lines[i][1];
        int x2 = lines[i][2];
        int y2 = lines[i][3];

        float slope;
        //Calculate slope
        if (x2 - x1 == 0) //corner case, avoiding division by 0
            slope = 999.0;
        else
            slope = (y2 - y1) / (float)(x2 - x1);

        if (abs(slope) > slope_threshold) { //degree 45
            slopes.push_back(slope);
            new_lines.push_back(lines[i]);
        }
    }

    for (int i = 0; i < new_lines.size(); i++)
    {
        Vec4i line = new_lines[i];
        float slope = slopes[i];

        int x1 = line[0];
        int y1 = line[1];
        int x2 = line[2];
        int y2 = line[3];

        if (slope > 0 && x1 > XHalf && x2 > XHalf)
            right_lines.push_back(line);
        else if (slope < 0 && x1 < XHalf && x2 < XHalf)
            left_lines.push_back(line);

    }

}

bool find_line_params(vector<Vec4i> &left_lines, float *left_m, float *left_b)
{
    float left_avg_x = 0, left_avg_y = 0, left_avg_slope = 0;

    if(left_lines.size() == 0)
        return false;
    
    for (int i = 0; i < left_lines.size(); i++)
    {
        left_avg_x += left_lines[i][0];
        left_avg_x += left_lines[i][1];
        left_avg_y += left_lines[i][2];
        left_avg_y += left_lines[i][3];
        left_avg_slope += (left_lines[i][3] - left_lines[i][1]) / (float)(left_lines[i][2] - left_lines[i][0]);
    }
    left_avg_x = left_avg_x / (left_lines.size() * 2);
    left_avg_y = left_avg_y / (left_lines.size() * 2);
    left_avg_slope = left_avg_slope / left_lines.size();

    //return values
    *left_m = left_avg_slope;
    //b=y-mx // find Y 
    *left_b = left_avg_y - left_avg_slope * left_avg_x;

    return true;

}

void find_lines(Mat &img, vector<cv::Vec4i> &left_lines, vector<Vec4i>& right_lines, float *lane_center, Point &vanishing_pt, float *leftX_inter, float *rightX_inter)
{
    static float left_slope_mem = -1, right_slope_mem = 1, right_b_mem = 0, left_b_mem = 0;

    float left_b, right_b, left_m, right_m;

    bool draw_left = find_line_params(left_lines, &left_m, &left_b);
    if(draw_left) {
        float left_x0 = (-left_b) / left_m;
        float left_x120 = (YHalf - left_b) / left_m;
        left_slope_mem = left_m;
        left_b_mem = left_b;
#ifdef CAMERA_SHOW
        line(img, Point(left_x0, 0), Point(left_x120, YHalf), Blue, 3);
        cout << left_lines.size() << " left lines,";
        *leftX_inter = left_x120;
#endif
    }
    else {
        cout << "\tNo Left Line,";
    }

    bool draw_right = find_line_params(right_lines, &right_m, &right_b);
    if (draw_right) {
        float right_x0 = (-right_b) / right_m;
        float right_x120 = (YHalf - right_b) / right_m;
        right_slope_mem = right_m;
        right_b_mem = right_b;
#ifdef CAMERA_SHOW
        line(img, Point(right_x0, 0), Point(right_x120, YHalf), Red, 3);
        *rightX_inter = right_x120;
#endif
        cout << right_lines.size() << " right lines," <<endl;
    }
    else {
        cout << "\tNo Right Line," << endl;
    }

    float left_xPt[2];
    float right_xPt[2];

    left_xPt[LOW] = ((YPoint[LOW] - left_b_mem) / left_slope_mem);
    right_xPt[LOW] = ((YPoint[LOW] - right_b_mem) / right_slope_mem);
    // ldistance[LOW] = XHalf - left_xPt[LOW]
    // rdistance[LOW] = right_xPt[LOW] - XHalf;

    left_xPt[HIGH] = ((YPoint[HIGH]- left_b_mem) / left_slope_mem);
    right_xPt[HIGH] = ((YPoint[HIGH]- right_b_mem) / right_slope_mem);
    // ldistance[LOW] = XHalf - left_xPt[HIGH]
    // rdistance[HIGH] = right_xPt[HIGH] - XHalf;

    lane_center[LOW] = (left_xPt[LOW] + right_xPt[LOW]) / 2.0;
    lane_center[HIGH] = (left_xPt[HIGH] + right_xPt[HIGH]) / 2.0;

    float Vx = -(left_b_mem - right_b_mem) / (left_slope_mem - right_slope_mem);
    float Vy = left_slope_mem * Vx + left_b_mem;
    vanishing_pt = Point(Vx, Vy);

}

int img_process(Mat &frame) //frame = input  output = Math
{
    static float leftX=0, rightX=0;  //frame -> grayframe -> roi(rect) ->
    Mat grayframe, edge_frame, roi_gray_ch3;
    Mat roi;
    //cvtColor(frame, grayframe, COLOR_BGR2GRAY); 
    Rect rect_roi(0, YHalf, Width, YHalf);
    roi = frame(rect_roi);

    cvtColor(roi, grayframe, COLOR_BGR2GRAY);
    GaussianBlur(grayframe, grayframe, Size(3,3), 1.5);
    cvtColor(grayframe, roi_gray_ch3, COLOR_GRAY2BGR); //color input  
    Canny(grayframe,edge_frame,70,150,3);

    vector<cv::Vec4i> lines_set;

    cv::HoughLinesP(edge_frame, lines_set, 1, PI/180,30,30,10); //lines output

#ifdef CAMERA_SHOW_MORE
    show_lines(roi_gray_ch3, lines_set); //draw lines
#endif
    vector<Vec4i> right_lines;
    vector<Vec4i> left_lines;
    split_left_right(lines_set, left_lines, right_lines);    //2set lines 
#ifdef CAMERA_SHOW
    show_lines(roi, left_lines, Sky, 2); //roi input lines
    show_lines(roi, right_lines, Pink, 2);
#endif
    float lane_center[2]; //ldistance[2], rdistance[2];
    Point vpt;

    find_lines(roi, left_lines, right_lines, lane_center, vpt, &leftX, &rightX); //rdistance, ldistance; vpt is venished point

    float ratio = (rightX-vpt.x) / (vpt.x-leftX); //ratio == 1 -> gree

#ifdef CAMERA_SHOW

    if (ratio > 1.1) //+-0.1 
        arrowedLine(frame, Point(XHalf, YHalf), Point(XHalf - ratio*10, YHalf), Scalar(255, 120, 40), 4 ); //X,YHalf = center point
    else if (ratio < 0.9)
        arrowedLine(frame, Point(XHalf, YHalf), Point(XHalf + 1.0/ratio*10 , YHalf), Scalar(40, 120, 255), 4);
    else //a == b left == right 
        circle(frame, Point(vpt.x, YHalf - 3), 5, Scalar(255, 70, 255), -1);

    int differ[2];
    for(int i = LOW; i <= HIGH; i++){
        differ[i] = lane_center [i] - XHalf;
        circle(roi, Point(XHalf, YPoint[i]), 5, Scalar(250, 250, 250), -1);
        circle(roi, Point(lane_center[i], YPoint[i]), 5, Scalar(0, 0, 255), 2);
        putText(roi, format("%3d - %3d = %3d", (int)lane_center[i], (int)XHalf, differ[i]), Point(XHalf- 100, YHalf / 2 + i *30), FONT_HERSHEY_SIMPLEX, 0.5, Yellow, 2);
    }
    circle(roi, Point(vpt.x, YHalf), 7, Scalar(255, 0 ,120), 3);
    imshow("roi", roi);
    imshow("edgeframe", edge_frame);
#endif    
#ifdef CAMERA_SHOW_MORE
    imshow("frame", frame);
    imshow("roi_gray_ch3", roi_gray_ch3);
#endif
    return ratio * 100;
}

int main(int argc, char**argv)
{
#ifdef ROS
 VideoCapture cap(0);
#else
  VideoCapture cap("comete.mp4");
#endif
  Mat frame;

  if(!cap.isOpened()){
    std::cout<<"No camera or No file!"<< std::endl;
    return -1;
  }

#ifdef ROS
  ros::init(argc, argv, "cam_msg_publisher");
  ros::NodeHandle nh;
  std_msgs::Int16 cam_msg;
  ros::Publisher pub = nh.advertise<std_msgs::int16>("cam_msg",100);

  int init_past=1;
  //--------------------------------------
  ros::Rate loop_rate(50);
  cout<<"Start"<<endl;
#endif

  int differ , key, fr_no = 0;
  bool capture = true;
  //clock_t tStart = clock();
  for(;;){
    if (capture) {
        cap >> frame;  //read file
        if (frame.empty())
            break;

    }

    if ((key = waitKey(30)) >= 0 ) {
        if (key == 27)
            break;
        else if (key =' ') {
            capture = !capture;
        }
    }

    if (capture == false)
        continue; //next frame
    
    fr_no++;
    resize(frame,frame,Size(Width,Height));
    differ = img_process(frame);

#ifdef ROS
    cam_msg.data = differ;  //differ is motor controller (steering & move)
    pub.publish(cam_msg);
     loop_rate.sleep();
#else
    std::cout << fr_no << ":" << differ << endl; //print differ
#endif
  }

  std::cout<<"Camera off"<<endl;
  return 0;

}
