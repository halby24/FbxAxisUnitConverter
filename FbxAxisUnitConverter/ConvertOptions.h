#pragma once
#include <string>
#include <optional>
#include <fbxsdk.h>

struct AxisVector
{
    int axis = 1;  // 0=X, 1=Y, 2=Z
    int sign = 1;  // +1 or -1

    bool IsOrthogonalTo(const AxisVector& other) const
    {
        return axis != other.axis;
    }
};

// ---------------------------------------------------------------------------
// ConversionParams — 変換ロジックが必要とする純粋なパラメータ
// FbxAxisUnitConverter ライブラリ利用者は、これと FbxScene* を渡すだけで変換できる。
// ---------------------------------------------------------------------------
struct ConversionParams
{
    std::optional<AxisVector> dstUp;
    std::optional<AxisVector> dstForward;

    std::optional<AxisVector> srcUp;
    std::optional<AxisVector> srcForward;

    std::optional<FbxSystemUnit> srcUnit;
    std::optional<FbxSystemUnit> dstUnit;

    // Pre-normalization: ルートオブジェクトに焼き込まれた補正変換を除去する
    std::optional<AxisVector>    preNormUp;
    std::optional<AxisVector>    preNormForward;
    std::optional<FbxSystemUnit> preNormUnit;
};

// ---------------------------------------------------------------------------
// CliOptions — CLI 専用オプション。ファイルパスと変換パラメータを保持する。
// ライブラリ利用者は ConversionParams を直接構築する。
// ---------------------------------------------------------------------------
struct CliOptions
{
    std::string      inputPath;
    std::string      outputPath;
    ConversionParams params;
};
