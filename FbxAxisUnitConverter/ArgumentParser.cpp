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
              << "  --src-unit    <unit>   Override input unit\n"
              << "\nPre-normalization (removes baked correction from root objects, e.g. Blender export):\n"
              << "  --pre-norm-up      <axis>   Effective source Up axis before baking (e.g. Z)\n"
              << "  --pre-norm-forward <axis>   Effective source Forward axis before baking (e.g. Y)\n"
              << "  --pre-norm-unit    <unit>   Effective source unit before baking (e.g. m)\n"
              << "  Note: --pre-norm-up/--pre-norm-forward must be specified together.\n"
              << "        Cannot be combined with --src-up/--src-forward/--src-unit.\n";
}

CliOptions ArgumentParser::Parse(int argc, char* argv[])
{
    CliOptions       cli;
    ConversionParams& p = cli.params;

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
            cli.inputPath = getNext(i);
        else if (arg == "-o" || arg == "--output")
            cli.outputPath = getNext(i);
        else if (arg == "--up")
            p.dstUp = ParseAxisVector(getNext(i));
        else if (arg == "--forward")
            p.dstForward = ParseAxisVector(getNext(i));
        else if (arg == "--unit")
            p.dstUnit = ParseUnit(getNext(i));
        else if (arg == "--src-up")
            p.srcUp = ParseAxisVector(getNext(i));
        else if (arg == "--src-forward")
            p.srcForward = ParseAxisVector(getNext(i));
        else if (arg == "--src-unit")
            p.srcUnit = ParseUnit(getNext(i));
        else if (arg == "--pre-norm-up")
            p.preNormUp = ParseAxisVector(getNext(i));
        else if (arg == "--pre-norm-forward")
            p.preNormForward = ParseAxisVector(getNext(i));
        else if (arg == "--pre-norm-unit")
            p.preNormUnit = ParseUnit(getNext(i));
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
    if (cli.inputPath.empty())
        throw std::runtime_error("Missing required argument: -i / --input");
    if (cli.outputPath.empty())
        throw std::runtime_error("Missing required argument: -o / --output");

    // --up/--forward は両方指定するか、両方省略する
    if (p.dstUp.has_value() != p.dstForward.has_value())
        throw std::runtime_error("--up and --forward must both be specified or both omitted.");
    if (p.srcUp.has_value() != p.srcForward.has_value())
        throw std::runtime_error("--src-up and --src-forward must both be specified or both omitted.");

    // 直交チェック（両方指定されている場合のみ）
    if (p.dstUp.has_value() && p.dstForward.has_value())
    {
        if (!p.dstUp->IsOrthogonalTo(*p.dstForward))
        {
            const char* axisNames[] = { "X", "Y", "Z" };
            std::string upStr   = (p.dstUp->sign < 0 ? "-" : "") + std::string(axisNames[p.dstUp->axis]);
            std::string fwdStr  = (p.dstForward->sign < 0 ? "-" : "") + std::string(axisNames[p.dstForward->axis]);
            throw std::runtime_error(
                "Error: --up and --forward vectors must be orthogonal. Got up=" + upStr + ", forward=" + fwdStr + ".");
        }
    }

    // 入力側も同様にチェック
    if (p.srcUp.has_value() && p.srcForward.has_value())
    {
        if (!p.srcUp->IsOrthogonalTo(*p.srcForward))
        {
            const char* axisNames[] = { "X", "Y", "Z" };
            std::string upStr  = (p.srcUp->sign < 0 ? "-" : "") + std::string(axisNames[p.srcUp->axis]);
            std::string fwdStr = (p.srcForward->sign < 0 ? "-" : "") + std::string(axisNames[p.srcForward->axis]);
            throw std::runtime_error(
                "Error: --src-up and --src-forward vectors must be orthogonal. Got up=" + upStr + ", forward=" + fwdStr + ".");
        }
    }

    // --pre-norm-up/--pre-norm-forward は両方指定するか、両方省略する
    if (p.preNormUp.has_value() != p.preNormForward.has_value())
        throw std::runtime_error("--pre-norm-up and --pre-norm-forward must both be specified or both omitted.");

    // pre-norm 軸の直交チェック
    if (p.preNormUp.has_value() && p.preNormForward.has_value())
    {
        if (!p.preNormUp->IsOrthogonalTo(*p.preNormForward))
        {
            const char* axisNames[] = { "X", "Y", "Z" };
            std::string upStr  = (p.preNormUp->sign < 0 ? "-" : "") + std::string(axisNames[p.preNormUp->axis]);
            std::string fwdStr = (p.preNormForward->sign < 0 ? "-" : "") + std::string(axisNames[p.preNormForward->axis]);
            throw std::runtime_error(
                "Error: --pre-norm-up and --pre-norm-forward vectors must be orthogonal. Got up=" + upStr + ", forward=" + fwdStr + ".");
        }
    }

    // --pre-norm-* と --src-* は排他
    bool hasPreNorm    = p.preNormUp.has_value() || p.preNormUnit.has_value();
    bool hasSrcOverride = p.srcUp.has_value() || p.srcUnit.has_value();
    if (hasPreNorm && hasSrcOverride)
        throw std::runtime_error("--pre-norm-* and --src-* options cannot be used together.");

    return cli;
}
