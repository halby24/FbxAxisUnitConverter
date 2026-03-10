#pragma once
#include "ConvertOptions.h"
#include "FbxFileIO.h"
#include <fbxsdk.h>

class FbxAxisUnitConverter
{
public:
    explicit FbxAxisUnitConverter(const ConvertOptions& options);
    ~FbxAxisUnitConverter();

    // メイン処理。成功時は 0 を返す
    int Run();

private:
    void PreNormalize();
    void BuildConversionMatrix();
    void ApplyGlobalSettings();
    void ProcessScene();

    ConvertOptions mOptions;  // value型（PreNormalize() で srcUp/srcForward/srcUnit を上書き可能）
    FbxManager*  mManager = nullptr;
    FbxScene*    mScene   = nullptr;

    FbxAMatrix   mConvMatrix;   // 変換行列（回転・スウィズル）
    double       mScaleFactor = 1.0;
    bool         mFlipWinding = false;
};
