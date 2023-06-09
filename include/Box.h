//
// Created by Qi on 22-10-13.
//


#ifndef BOX_H
#define BOX_H

#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>


#include <opencv2/features2d.hpp>
#include "MapPoint.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense> 

namespace ORB_SLAM2 {

typedef Eigen::Matrix<double, 3, 1> Vec3;
typedef Eigen::Matrix<double, 2, 1> Vec2;


struct Frame;
struct MapPoint;

/**
 * 有一个新帧的时候关联一个boxes
 */
struct Box{
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    

//     std::weak_ptr<Frame> first_frame_;         // 持有该box的frame
//     std::vector<Frame::Ptr> frames_;
    std::vector<int>frames_id_;                  //持有该box的frame's id
//     int initial_frame_id_;   
    // Frame::Ptr initial_frame_;    
                        

    unsigned long id_;                              // id
    // for evaluation
    float id4eval_;
    // for evaluation**
    std::string obj_type_;               // object type
    Vec2 left_top_2D_, right_bottom_2D_; //2D box的两个点
    double h_, w_, l_;                      //3Dbox的长宽高  
    Vec3 center_3D_;                     //3D object center
    Vec3 filtered_center_3D_;            //Kalman
    double direction_;
    Vec3 head_direction_;
    Vec3 predicted_direction_;
    Vec3 filtered_predicted_direction;   //Kalman
    double predicted_distance_;

    double v_, a_, aa_;  //每一个物体的速度，加速度，加速度的加速度
    bool isStatic_ = false;
    std::vector<double> v_list_;


    std::vector<Vec3> trajactory_;
    std::vector<Vec3> trajactory_optimized_;

    //kalman filter
    cv::KalmanFilter KF_;
    double dt_ = 0.01;
    // cv::Mat measurements_;

    std::vector<double> directions_;









   public:
    Box(){}
    Box(long id, std::string obj_type, Vec2 left_top_2D, Vec2 right_bottom_2D,
            double h, double w, double l, Vec3 center_3D, double direction)
            : id_(id), obj_type_(obj_type), left_top_2D_(left_top_2D), right_bottom_2D_(right_bottom_2D),
            h_(h), w_(w), l_(l), center_3D_(center_3D), direction_(direction){}

    void Setvaaa();
    void PredictTrajactory();
    void CalculateDistance();
    void CalculateHeadDirection();
    void CalculatePredictedDirection();

    Vec3 normalize(Vec3 vector);
    double angle(Vec3 vector);
 

    };
}  // namespace myslam

#endif  // MYSLAM_BOX_H