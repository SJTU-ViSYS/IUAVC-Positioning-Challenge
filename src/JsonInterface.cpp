#include "JsonInterface.h"
#include "../thirdparty/rapidjson/include/rapidjson/writer.h"
#include "../thirdparty/rapidjson/include/rapidjson/stringbuffer.h"
#include <ros/ros.h>
#include <cstdio>
JsonInterface::JsonInterface(){

}

bool JsonInterface::OpenJsonfile(const char* filePath){
    mFileStr=filePath;
    std::ifstream fileStream;
    fileStream.open(filePath);
    if(!fileStream.is_open()){
        return false;
    }
    fileStream.seekg(0 , std::ios::end);  //将指针指向文件的结尾
 
    int nLen = fileStream.tellg();   //获取文件的长度
 
    fileStream.seekg(0 , std::ios::beg);  //再将指针指向文件的开始，主要是为了读取数据
 
    char* json = new char[nLen];
 

    fileStream.read(json,nLen);
    // 1. 把 JSON 解析至 DOM。
    mDoc.Parse(json);
    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    mDoc.Accept(writer);
    ROS_INFO("%s",buffer.GetString());
    return true;
}

bool JsonInterface::OpenJsonfile(std::string& filePath){
    mFileStr=filePath;
    std::ifstream fileStream;
    fileStream.open(filePath.c_str());
    if(!fileStream.is_open()){
        ROS_ERROR("Wrong Config File.");
        return false;
    }
    fileStream.seekg(0, std::ios::end);  //将指针指向文件的结尾
 
    int nLen = fileStream.tellg();   //获取文件的长度
 
    fileStream.seekg(0 , std::ios::beg);  //再将指针指向文件的开始，主要是为了读取数据
 
    char* json = new char[nLen];
 
    fileStream.read(json,nLen);
    ROS_INFO("%s",json);
    // 1. 把 JSON 解析至 DOM。
    mDoc.Parse(json);
    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    mDoc.Accept(writer);
    ROS_INFO("%s",buffer.GetString());
    return true;
}

bool JsonInterface::UpdateJsonfile(){
    std::fstream fileStream;
    fileStream.open(mFileStr,std::ios::app|std::ios::out);
    if(!fileStream.is_open()){
        return false;
    }
    fileStream.seekg(0 , std::ios::beg);
    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    mDoc.Accept(writer);
    fileStream<<buffer.GetString()<<std::endl;
    return true;
}
JsonInterface::~JsonInterface(){
}
bool JsonInterface::GetParam(const std::string& param, int &i){
    rapidjson::Value& val = mDoc[param.c_str()];
    if(!val.IsInt()){
        return false;
    }
    i=val.GetInt();
    return true;
}
bool JsonInterface::GetParam(const std::string& param, std::string &str){
    rapidjson::Value& val = mDoc[param.c_str()];
    if(!val.IsString()){
        return false;
    }
    str=val.GetString();
    return true;
}
bool JsonInterface::GetParam(const std::string& param, bool &b){
    rapidjson::Value& val = mDoc[param.c_str()];
    if(!val.IsBool()){
        return false;
    }
    b=val.GetBool();
    return true;
}
bool JsonInterface::GetParam(const std::string& param, float &f){
    rapidjson::Value& val = mDoc[param.c_str()];
    if(!val.IsFloat()){
        return false;
    }
    f=val.GetFloat();
    return true;
}

bool JsonInterface::SetParam(const std::string& param, const int &i){
    rapidjson::Value& val = mDoc[param.c_str()];
    val.SetInt(i);

    return true;
}
bool JsonInterface::SetParam(const std::string& param, const std::string &str){
    rapidjson::Value& val = mDoc[param.c_str()];
    val.SetString(str.c_str(),str.length());
    return true;
}
bool JsonInterface::SetParam(const std::string& param, const bool &b){
    rapidjson::Value& val = mDoc[param.c_str()];
    val.SetBool(b);
    return true;
}
bool JsonInterface::SetParam(const std::string& param, const float &f){
    rapidjson::Value& val = mDoc[param.c_str()];
    val.SetFloat(f);
    return true;
}
bool JsonInterface::SetParam(const std::string& param, const double &d){
    rapidjson::Value& val = mDoc[param.c_str()];
    val.SetDouble(d);
    return true;
}