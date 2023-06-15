//
// Created by wh on 23-2-17.
//

#ifndef OBJMODEL_H
#define OBJMODEL_H


#include <pangolin/pangolin.h>
#include <opencv2/opencv.hpp>

#include <pangolin/gl/glformattraits.h>


namespace ORB_SLAM2{
    struct Float3       //点的位置信息和法线信息的数据类型
    {
        float Data[3];     //x,y,z
    };

    struct Float2      //点的纹理坐标数据类型
    {
        float Data[2];   //u,v
    };

    struct Face          //面信息
    {
        int vertex[3][3];       //三个点构成一个面  每个点有三个索引信息
    };
    class ObjModel
    {
    public:
        ObjModel(){};
        ObjModel(const char * objFileName);
        //void setTextureFromBmp(const char *texFileName);  //从obj文件创建纹理
        void ObjDrawCar(float angle);//画图
        void ObjDrawHuman(float angle);//画图
        void Filter(int n);//过滤面
        unsigned char*LoadFileContent(const char*path, int &filesize);//读取文件，返回文件内容，把文件大小赋值给filesize
        //ObjModel::ObjModel(const char * objFileName);//根据.obj文件的格式读取文件中的数据

        std::vector<Float3> mLocation;   //位置信息
        std::vector<Float3> mNormal;     //法线信息
        std::vector<Float2> mTexcoord;   //纹理坐标信息
        std::vector<Face> mFace;         //面信息
        GLuint mTexture;            //模型纹理
        std::vector<Face> filter_Face;  //过滤之后的面信息
        // float angle_ = 0;
    };

}

#endif //OBJMODEL_H