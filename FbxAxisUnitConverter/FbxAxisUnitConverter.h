#pragma once
#include "ConvertOptions.h"
#include "Logger.h"
#include <fbxsdk.h>

// ---------------------------------------------------------------------------
// FbxAxisUnitConverter — 軸・単位変換コア（ライブラリAPI）
//
// 呼び出し側が用意した FbxScene を直接書き換える。
// FbxManager / シーンの所有権、ファイル I/O は一切持たない。
//   - シーン構築・破棄は呼び出し側の責務
//   - ファイル読み書きが必要なら FbxFileIO を使う
//   - ログ出力先は ILogger* で注入。nullptr のときは出力なし
// ---------------------------------------------------------------------------
class FbxAxisUnitConverter
{
public:
    // scene  : 変換対象のシーン（非所有）
    // params : 変換パラメータ
    // logger : 出力先（非所有, nullptr 可）
    FbxAxisUnitConverter(FbxScene* scene,
                         const ConversionParams& params,
                         ILogger* logger = nullptr);

    // 変換を適用する。失敗時は std::runtime_error をスロー。
    void Convert();

private:
    void PreNormalize();
    void BuildConversionMatrix();
    void ApplyGlobalSettings();
    void ProcessScene();

    FbxScene*        mScene;   // 非所有
    ConversionParams mParams;  // value型（PreNormalize() で srcUp/srcForward/srcUnit を上書き可能）
    ILogger*         mLogger;  // 非所有, 内部で nullptr のときは NullLogger にフォールバック

    FbxAMatrix mConvMatrix;    // 変換行列（回転・スウィズル）
    double     mScaleFactor = 1.0;
    bool       mFlipWinding = false;
};
