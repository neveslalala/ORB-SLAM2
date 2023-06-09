//
// Created by Qi on 13-10-2022.
//

#include "Box.h"
#include <math.h> 
#include <cmath>
using namespace std;

namespace ORB_SLAM2 {



void Box::PredictTrajactory(){

    /* bazier curve
        1. Calculate v and a to get predicted distance.
        2. Calculate head direction.
        3. Calculate predicted direction.
        4. Calculate curve extend (here use half norm).   */
    //initialize 
    v_ = 0;
    
    Setvaaa();
    CalculateHeadDirection();
    CalculatePredictedDirection();
    CalculateDistance();

    
    
}


void Box::Setvaaa(){
    int n = Config::Get<int>("t") * 10;
    int size = trajactory_optimized_.size();
    
    Vec3 point1 = trajactory_optimized_[size - 1];
    Vec3 point2 = trajactory_optimized_[size - 6];
    Vec3 point3 = trajactory_optimized_[size - 20];

    // // for evaluation
    // Vec3 point1 = trajactory_optimized_[size - 1 - n];
    // Vec3 point2 = trajactory_optimized_[size - 6 - n];
    // Vec3 point3 = trajactory_optimized_[size - 20 - n];
    // //for evaluation **
    double v1, v2, a;

    v1 = sqrt(pow((point1.x() - point2.x()), 2) + pow((point1.y() - point2.y()), 2)
                + pow((point1.z() - point2.z()), 2)) / 0.5;
    // v_ = v1;



    double v = 0;
  
    if(size > 20){
        for(int i = 0 ; i < 10 ; i++){
        
            v += sqrt(pow((trajactory_optimized_[size - 11 - i].x() - trajactory_optimized_[size - 1 - i].x()), 2) 
                    + pow((trajactory_optimized_[size - 11 - i].z() - trajactory_optimized_[size - 1 - i].z()), 2));
        }
        v_ = v / 10;

    }
    else if (size > 10)
    {
        for (int i = 0 ; i < 5 ; i++){
            v += sqrt(pow((trajactory_optimized_[size - 6 - i].x() - trajactory_optimized_[size - 1 - i].x()), 2) 
                    + pow((trajactory_optimized_[size - 6 - i].z() - trajactory_optimized_[size - 1 - i].z()), 2));
        }
        v_ = v / 2.5;
    }
    
    

}


void Box::CalculateDistance(){

    double t = Config::Get<double>("t");
    // int size = trajactory_optimized_.size();
    // double t = 2;
    // predicted_distance_ = abs(v_ * t);
    // predicted_distance_ = 2;
    // std::cout << "v_: " << v_ << endl;
    // std::cout << "predicted_distance: " << predicted_distance_ << std::endl;

    double curve_length = abs(v_ * t);
    predicted_distance_ = curve_length;
    double theta = calculateRad(head_direction_, predicted_direction_);
    if(abs(theta) < 20 * 3.1415 * 2 / 360){
        predicted_distance_ = curve_length;
    }else{
        double x = cos(3.1415 - 2 * theta) + curve_length / 2 * theta;
        double y = sin(3.1415 - 2 * theta);
        predicted_distance_ = sqrt(x * x + y * y);
    }

}

void Box::CalculateHeadDirection(){
  
    int n = Config::Get<int>("t") * 10;
    

    int size = trajactory_optimized_.size();
    // double t = 2;
    
     Vec3 point1 = trajactory_optimized_[size - 4];
    Vec3 point2 = trajactory_optimized_[size - 10];
    // //for evaluation
    // Vec3 point1 = trajactory_optimized_[size - 1 - n];
    // Vec3 point2 = trajactory_optimized_[size - 5 - n];
    // //for evaluation*
    Vec3 norm_vector = point1 - point2;
    //方向向量的z轴可以设置成0
    norm_vector.y() = 0;
    //单位化前进方向法向量
    Vec3 normalize_norm_vector = normalize(norm_vector) ;
    
    //单位化3Dbox方向向量
    Vec3 direction;
    //x1 =  x*cosθ- y * sinθ     y1 = y*cosθ + x * sinθ （1， 0， 0）是初始向量
    direction.x() = cos(direction_) * (1) - sin(direction_) * 0;
    direction.y() = 0;
    direction.z() = cos(direction_) * 0 - sin(direction_) * (1);

    //两个方向向量夹角太大(cos<a,b>=a.b/|a||b|, 夹角为锐角时，cosθ>0；夹角为钝角时,cosθ<0.)
    if (((normalize_norm_vector.x() * direction.x() + normalize_norm_vector.z() * direction.z()) / 
            sqrt(pow((normalize_norm_vector.x()), 2) + pow((normalize_norm_vector.z()), 2)) * 
            sqrt(pow((direction.x()), 2) + pow((direction.z()), 2))) > 0){
        head_direction_ = normalize_norm_vector;
        std::cout << head_direction_.x() << " " << head_direction_.y() << " " <<head_direction_.z() << std::endl;
        
        head_direction_ = normalize(head_direction_);
        std::cout << head_direction_.x() << " " << head_direction_.y() << " " <<head_direction_.z() << std::endl;
    }
    else{
        //预测的方向是前进方向法向量和3Dbox方向向量的权重加法。（这里的权重是1：1）
        head_direction_ = 0.000001 * direction + 0.99 * normalize_norm_vector;
        std::cout << head_direction_.x() << " " << head_direction_.y() << " " <<head_direction_.z() << std::endl;

        //normalize predicted direction
        head_direction_ = normalize(head_direction_);
        std::cout << head_direction_.x() << " " << head_direction_.y() << " " <<head_direction_.z() << std::endl;

    }
    
}

void Box::CalculatePredictedDirection(){
    double t = Config::Get<double>("t");
    int size = trajactory_optimized_.size();
    int n = Config::Get<int>("t") * 10;

    if(size < 20){
    // //for evaluation 
    // if(size < 20 - n){
    // //for evaluation**
        predicted_direction_ = head_direction_;
    }else{

        Vec3 point1 = trajactory_optimized_[size - 3];   //20 / 3  = 10 间隔是6
        Vec3 point2 = trajactory_optimized_[size - 11];
        Vec3 point3 = trajactory_optimized_[size - 19];
       
       
    // //    for evaluation
    //      // 方法1： 在前20帧内均匀取2个点
    //     Vec3 point1 = trajactory_optimized_[size - 1 - n];   //20 / 3  = 10 间隔是6
    //     Vec3 point2 = trajactory_optimized_[size - 7 - n];
    //     Vec3 point3 = trajactory_optimized_[size - 13 - n];
    //     //for evaluation **


        //求出来四个方向向量
        Vec3 norm_vector1 = normalize(point2 - point1);
        Vec3 norm_vector2 = normalize(point3 - point2);
        predicted_direction_ = -2 * norm_vector1 + norm_vector2;


 


        // Vec3 norm_vector3 = normalize(point3 - point4);
        // Vec3 norm_vector4 = normalize(point4 - point5);
        // Vec3 norm_vector4 = normalize(point4 - point44);
        // Vec3 norm_vector4 = normalize(point4 - point44);
        // double acosx1 = acos(norm_vector3.x()) - 3 * acos(norm_vector2.x()) + 3 * acos(norm_vector1.x());
        // predicted_direction_.x() = cos(acosx1);

        // double acosz1 = acos(norm_vector3.z()) - 3 * acos(norm_vector2.z()) + 3 * acos(norm_vector1.z());
        // predicted_direction_.z() = cos(acosz1);


        // double acosx1 = 2 * acos(norm_vector1.x()) -  acos(norm_vector2.x());
        // predicted_direction_.x() = cos(acosx1);
        

        // double acosz1 = 2 * acos(norm_vector1.x()) - acos(norm_vector2.z());
        // predicted_direction_.z() = cos(acosz1);

        // if(norm_vector1.x() > 0) {
        //     predicted_direction_.x() = abs(predicted_direction_.x());
        // }else{
        //     predicted_direction_.x() = -abs(predicted_direction_.x());
        // }

        // if(norm_vector1.z() > 0) {
        //     predicted_direction_.z() = abs(predicted_direction_.z());
        // }else{
        //     predicted_direction_.z() = -abs(predicted_direction_.x());
        // }





        // double angle1, angle2, angle3, difference1, difference2, result_angle, difference;
        // predicted_direction_ = 0.7 * predicted_direction_ + 0.1 * norm_vector3 + 0.2 * head_direction_;

        // predicted_direction_ = norm_vector3 + 2 * norm_vector2 - norm_vector1 - norm_vector4;
        predicted_direction_ = normalize(predicted_direction_);
        // predicted_direction_ = 0.5 * head_direction_ + 0.5 * predicted_direction_;


    }
  
}


Vec3 Box::normalize(Vec3 vector){
    return  vector / sqrt( pow((vector.x()), 2) + pow((vector.z()), 2));
}

//与x轴正向的夹角余弦值：cosa=x/sqrt(x^2+y^2)
double Box::angle(Vec3 vector){
    double angle;
    if(vector.z() > 0){ //正前方计算与-x的夹角并+3.14
        angle = 3.1415926 + acos(-vector.x() / sqrt(pow((vector.x()), 2) + pow((vector.z()), 2)));
    }
    if(vector.z() < 0){ //后方是与x轴的夹角
        angle = acos(vector.x() / sqrt(pow((vector.x()), 2) + pow((vector.z()), 2)));
    }
    return angle;
}


}

