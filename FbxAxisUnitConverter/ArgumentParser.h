#pragma once
#include "ConvertOptions.h"
#include <string>

class ArgumentParser
{
public:
    // 引数を解析して CliOptions を返す。失敗時は std::runtime_error を投げる
    static CliOptions Parse(int argc, char* argv[]);

private:
    static AxisVector ParseAxisVector(const std::string& str);
    static FbxSystemUnit ParseUnit(const std::string& str);
    static void PrintUsage(const char* programName);
};
