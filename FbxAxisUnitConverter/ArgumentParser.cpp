#include "ArgumentParser.h"
#include <stdexcept>
#include <algorithm>
#include <iostream>
#include <cstring>

static std::string ToLower(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(), ::tolower);
    return s;
}

AxisVector ArgumentParser::ParseAxisVector(const std::string& str)
{
    AxisVector v;
    std::string s = ToLower(str);

    if (s == "x")       { v.axis = 0; v.sign =  1; }
    else if (s == "-x") { v.axis = 0; v.sign = -1; }
    else if (s == "y")  { v.axis = 1; v.sign =  1; }
    else if (s == "-y") { v.axis = 1; v.sign = -1; }
    else if (s == "z")  { v.axis = 2; v.sign =  1; }
    else if (s == "-z") { v.axis = 2; v.sign = -1; }
    else
    {
        throw std::runtime_error("Invalid axis vector: '" + str + "'. Use X, -X, Y, -Y, Z, or -Z.");
    }
    return v;
}

FbxSystemUnit ArgumentParser::ParseUnit(const std::string& str)
{
    std::string s = ToLower(str);
    if      (s == "mm")   return FbxSystemUnit::mm;
    else if (s == "cm")   return FbxSystemUnit::cm;
    else if (s == "dm")   return FbxSystemUnit::dm;
    else if (s == "m")    return FbxSystemUnit::m;
    else if (s == "km")   return FbxSystemUnit::km;
    else if (s == "inch") return FbxSystemUnit::Inch;
    else if (s == "foot") return FbxSystemUnit::Foot;
    else if (s == "yard") return FbxSystemUnit::Yard;
    else if (s == "mile") return FbxSystemUnit::Mile;
    else
    {
        throw std::runtime_error(
            "Invalid unit: '" + str + "'. Use mm, cm, dm, m, km, inch, foot, yard, or mile.");
    }
}

void ArgumentParser::PrintUsage(const char* programName)
{
    std::cout << "Usage: " << programName << " -i <input.fbx> -o <output.fbx> [options]\n\n"
              << "Options:\n"
              << "  -i, --input   <path>   Input FBX file path (required)\n"
              << "  -o, --output  <path>   Output FBX file path (required)\n"
              << "  --up          <axis>   Output Up vector   (e.g. Y, Z, -Z)\n"
              << "  --forward     <axis>   Output Forward vector (e.g. -Z, X)\n"
              << "  --unit        <unit>   Output unit (mm|cm|dm|m|km|inch|foot|yard|mile)\n"
              << "  --src-up      <axis>   Override input Up vector\n"
              << "  --src-forward <axis>   Override input Forward vector\n"
              << "  --src-unit    <unit>   Override input unit\n";
}

ConvertOptions ArgumentParser::Parse(int argc, char* argv[])
{
    ConvertOptions opts;

    auto getNext = [&](int& i) -> std::string
    {
        if (i + 1 >= argc)
            throw std::runtime_error(std::string("Missing value for argument: ") + argv[i]);
        return argv[++i];
    };

    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];

        if (arg == "-i" || arg == "--input")
            opts.inputPath = getNext(i);
        else if (arg == "-o" || arg == "--output")
            opts.outputPath = getNext(i);
        else if (arg == "--up")
            opts.dstUp = ParseAxisVector(getNext(i));
        else if (arg == "--forward")
            opts.dstForward = ParseAxisVector(getNext(i));
        else if (arg == "--unit")
            opts.dstUnit = ParseUnit(getNext(i));
        else if (arg == "--src-up")
            opts.srcUp = ParseAxisVector(getNext(i));
        else if (arg == "--src-forward")
            opts.srcForward = ParseAxisVector(getNext(i));
        else if (arg == "--src-unit")
            opts.srcUnit = ParseUnit(getNext(i));
        else if (arg == "-h" || arg == "--help")
        {
            PrintUsage(argv[0]);
            std::exit(0);
        }
        else
        {
            throw std::runtime_error("Unknown argument: '" + arg + "'");
        }
    }

    // 必須引数チェック
    if (opts.inputPath.empty())
        throw std::runtime_error("Missing required argument: -i / --input");
    if (opts.outputPath.empty())
        throw std::runtime_error("Missing required argument: -o / --output");

    // --up/--forward は両方指定するか、両方省略する
    if (opts.dstUp.has_value() != opts.dstForward.has_value())
        throw std::runtime_error("--up and --forward must both be specified or both omitted.");
    if (opts.srcUp.has_value() != opts.srcForward.has_value())
        throw std::runtime_error("--src-up and --src-forward must both be specified or both omitted.");

    // 直交チェック（両方指定されている場合のみ）
    if (opts.dstUp.has_value() && opts.dstForward.has_value())
    {
        if (!opts.dstUp->IsOrthogonalTo(*opts.dstForward))
        {
            const char* axisNames[] = { "X", "Y", "Z" };
            std::string upStr   = (opts.dstUp->sign < 0 ? "-" : "") + std::string(axisNames[opts.dstUp->axis]);
            std::string fwdStr  = (opts.dstForward->sign < 0 ? "-" : "") + std::string(axisNames[opts.dstForward->axis]);
            throw std::runtime_error(
                "Error: --up and --forward vectors must be orthogonal. Got up=" + upStr + ", forward=" + fwdStr + ".");
        }
    }

    // 入力側も同様にチェック
    if (opts.srcUp.has_value() && opts.srcForward.has_value())
    {
        if (!opts.srcUp->IsOrthogonalTo(*opts.srcForward))
        {
            const char* axisNames[] = { "X", "Y", "Z" };
            std::string upStr  = (opts.srcUp->sign < 0 ? "-" : "") + std::string(axisNames[opts.srcUp->axis]);
            std::string fwdStr = (opts.srcForward->sign < 0 ? "-" : "") + std::string(axisNames[opts.srcForward->axis]);
            throw std::runtime_error(
                "Error: --src-up and --src-forward vectors must be orthogonal. Got up=" + upStr + ", forward=" + fwdStr + ".");
        }
    }

    return opts;
}
