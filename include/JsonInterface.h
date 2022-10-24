#include "../thirdparty/rapidjson/include/rapidjson/document.h"
#include <iostream>
#include <fstream>
#include <string>
class JsonInterface{
    public:
        JsonInterface();
        ~JsonInterface();
        bool OpenJsonfile(const char* filePath);
        bool OpenJsonfile(std::string& filePath);
        bool UpdateJsonfile();
        bool GetParam(const std::string& param, int &i);
        bool GetParam(const std::string& param, std::string &str);
        bool GetParam(const std::string& param, bool &b);
        bool GetParam(const std::string& param, double &d);
        bool GetParam(const std::string& param, float &f);

        bool SetParam(const std::string& param, const int &i);
        bool SetParam(const std::string& param, const std::string &str);
        bool SetParam(const std::string& param, const bool &b);
        bool SetParam(const std::string& param, const float &f);
        bool SetParam(const std::string& param, const double &d);
    private:
        std::string mFileStr;
        rapidjson::Document mDoc;
};
