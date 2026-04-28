#pragma once
#include "Logger.h"
#include <fbxsdk.h>
#include <set>

class GeometryProcessor
{
public:
    // logger は非所有, nullptr 可（その場合ログ出力なし）
    explicit GeometryProcessor(bool flipWinding, ILogger* logger = nullptr);

    void ProcessScene(FbxScene* scene, const FbxAMatrix& convMatrix, double scale);

private:
    void ProcessMesh(FbxMesh* mesh, const FbxAMatrix& convMatrix, const FbxAMatrix& convInv, double scale);
    void ProcessSkin(FbxMesh* mesh, const FbxAMatrix& convMatrix, const FbxAMatrix& convInv, double scale);
    void ProcessPoses(FbxScene* scene, const FbxAMatrix& convMatrix, const FbxAMatrix& convInv, double scale);
    void FlipWindingOrder(FbxMesh* mesh);

    bool       mFlipWinding;
    ILogger*   mLogger;
    std::set<FbxMesh*>    mProcessedMeshes;
    std::set<FbxCluster*> mProcessedClusters;
};
