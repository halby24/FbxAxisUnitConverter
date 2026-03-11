#include "GeometryProcessor.h"
#include <iostream>
#include <algorithm>
#include <vector>

// ---------------------------------------------------------------------------
// Helper: reverse per-polygon-vertex entries of a typed layer element
// Used when flipping winding order so layer data stays in sync with vertex order.
// ---------------------------------------------------------------------------
template<typename T>
static void ReverseLayerElementByPolygon(
    FbxLayerElementTemplate<T>* elem,
    const FbxMesh*               mesh)
{
    if (!elem) return;
    if (elem->GetMappingMode() != FbxLayerElement::eByPolygonVertex) return;

    int polyCount = mesh->GetPolygonCount();

    auto reverseRange = [&](auto& arr, int start, int sz) {
        for (int a = start + 1, b = start + sz - 1; a < b; ++a, --b)
        {
            auto tmp = arr.GetAt(a);
            arr.SetAt(a, arr.GetAt(b));
            arr.SetAt(b, tmp);
        }
    };

    if (elem->GetReferenceMode() == FbxLayerElement::eDirect)
    {
        auto& arr = elem->GetDirectArray();
        for (int pi = 0; pi < polyCount; ++pi)
        {
            int sz    = mesh->GetPolygonSize(pi);
            int start = mesh->GetPolygonVertexIndex(pi);
            if (sz >= 3) reverseRange(arr, start, sz);
        }
    }
    else if (elem->GetReferenceMode() == FbxLayerElement::eIndexToDirect)
    {
        auto& idx = elem->GetIndexArray();
        for (int pi = 0; pi < polyCount; ++pi)
        {
            int sz    = mesh->GetPolygonSize(pi);
            int start = mesh->GetPolygonVertexIndex(pi);
            if (sz >= 3) reverseRange(idx, start, sz);
        }
    }
}

// ---------------------------------------------------------------------------

GeometryProcessor::GeometryProcessor(bool flipWinding)
    : mFlipWinding(flipWinding)
{
}

void GeometryProcessor::ProcessScene(FbxScene* scene, const FbxAMatrix& convMatrix, double scale)
{
    // convInv は全メッシュ・ポーズで共通なので一度だけ計算する
    FbxAMatrix convInv = convMatrix.Inverse();

    int meshCount = scene->GetSrcObjectCount<FbxMesh>();
    for (int i = 0; i < meshCount; ++i)
    {
        FbxMesh* mesh = scene->GetSrcObject<FbxMesh>(i);
        ProcessMesh(mesh, convMatrix, convInv, scale);
    }

    // FbxPose (バインドポーズ) の変換
    // Blender はバインドポーズを TransformLinkMatrix より優先して参照するため、
    // 変換しないとボーンが元座標系のままになる
    ProcessPoses(scene, convMatrix, convInv, scale);
}

void GeometryProcessor::ProcessMesh(FbxMesh* mesh, const FbxAMatrix& convMatrix, const FbxAMatrix& convInv, double scale)
{
    if (mProcessedMeshes.count(mesh)) return;

    // --- Control points (vertex positions): rotate + scale ---
    int numVerts = mesh->GetControlPointsCount();
    FbxVector4* verts = mesh->GetControlPoints();
    #pragma omp parallel for
    for (int i = 0; i < numVerts; ++i)
    {
        FbxVector4 v(verts[i][0] * scale,
                     verts[i][1] * scale,
                     verts[i][2] * scale, 0.0);
        FbxVector4 vNew = convMatrix.MultT(v);
        verts[i] = FbxVector4(vNew[0], vNew[1], vNew[2], verts[i][3]);
    }

    // --- Layer elements: normals, tangents, binormals (rotate only, no scale) ---
    // eDirect / eIndexToDirect 両方に対応。
    // eIndexToDirect の場合も DirectArray に実体ベクトルが格納されているため同じ処理でよい。
    auto rotateDirectArray = [&convMatrix](FbxLayerElementTemplate<FbxVector4>* elem)
    {
        if (!elem) return;
        auto ref = elem->GetReferenceMode();
        if (ref != FbxLayerElement::eDirect && ref != FbxLayerElement::eIndexToDirect) return;

        auto& arr = elem->GetDirectArray();
        int cnt = arr.GetCount();
        auto* ptr = static_cast<FbxVector4*>(arr.GetLocked(FbxLayerElementArray::eReadWriteLock));
        if (ptr)
        {
            #pragma omp parallel for
            for (int i = 0; i < cnt; ++i)
            {
                FbxVector4& v = ptr[i];
                FbxVector4 vNew = convMatrix.MultT(FbxVector4(v[0], v[1], v[2], 0.0));
                v = FbxVector4(vNew[0], vNew[1], vNew[2], v[3]);
            }
            arr.Release(reinterpret_cast<void**>(&ptr));
        }
    };

    int layerCount = mesh->GetLayerCount();
    for (int li = 0; li < layerCount; ++li)
    {
        FbxLayer* layer = mesh->GetLayer(li);
        if (!layer) continue;

        if (auto* elem = layer->GetNormals())
        {
            if (elem->GetMappingMode() == FbxLayerElement::eByPolygonVertex ||
                elem->GetMappingMode() == FbxLayerElement::eByControlPoint)
            {
                rotateDirectArray(elem);
            }
        }

        rotateDirectArray(layer->GetTangents());
        rotateDirectArray(layer->GetBinormals());
    }

    // --- Skin deformers ---
    ProcessSkin(mesh, convMatrix, convInv, scale);

    // --- Winding order flip (when handedness changes) ---
    if (mFlipWinding)
        FlipWindingOrder(mesh);

    mProcessedMeshes.insert(mesh);
    std::cout << "[Geometry] Processed mesh: " << mesh->GetName() << "\n";
}

// ---------------------------------------------------------------------------
// Helper: convert a cluster bind-pose matrix to the new coordinate system.
//   M_new = convMatrix * M_old * convInv
//   Translation component is additionally multiplied by scale.
// ---------------------------------------------------------------------------
static FbxAMatrix ConvertClusterMatrix(
    const FbxAMatrix& mat,
    const FbxAMatrix& convMatrix,
    const FbxAMatrix& convInv,
    double scale)
{
    FbxAMatrix newMat = convMatrix * mat * convInv;
    FbxVector4 t = newMat.GetT();
    newMat.SetT(FbxVector4(t[0] * scale, t[1] * scale, t[2] * scale, t[3]));
    return newMat;
}

void GeometryProcessor::ProcessSkin(
    FbxMesh* mesh,
    const FbxAMatrix& convMatrix,
    const FbxAMatrix& convInv,
    double scale)
{
    int deformerCount = mesh->GetDeformerCount(FbxDeformer::eSkin);
    for (int di = 0; di < deformerCount; ++di)
    {
        FbxSkin* skin = static_cast<FbxSkin*>(mesh->GetDeformer(di, FbxDeformer::eSkin));
        if (!skin) continue;

        int clusterCount = skin->GetClusterCount();
        for (int ci = 0; ci < clusterCount; ++ci)
        {
            FbxCluster* cluster = skin->GetCluster(ci);
            if (!cluster) continue;
            if (mProcessedClusters.count(cluster)) continue;

            // TransformMatrix: mesh world transform at bind time
            FbxAMatrix tm;
            cluster->GetTransformMatrix(tm);
            cluster->SetTransformMatrix(ConvertClusterMatrix(tm, convMatrix, convInv, scale));

            // TransformLinkMatrix: bone world transform at bind time
            FbxAMatrix tlm;
            cluster->GetTransformLinkMatrix(tlm);
            cluster->SetTransformLinkMatrix(ConvertClusterMatrix(tlm, convMatrix, convInv, scale));

            // TransformParentMatrix (optional, only valid when set)
            if (cluster->IsTransformParentSet())
            {
                FbxAMatrix tpm;
                cluster->GetTransformParentMatrix(tpm);
                cluster->SetTransformParentMatrix(ConvertClusterMatrix(tpm, convMatrix, convInv, scale));
            }

            mProcessedClusters.insert(cluster);
            std::cout << "[Geometry] Processed cluster: " << cluster->GetName() << "\n";
        }
    }
}

// ---------------------------------------------------------------------------
// Helper: FbxMatrix <-> FbxAMatrix の相互変換 (SetRow 経由)
// FbxAMatrix のコンストラクターに FbxMatrix を渡す直接変換は SDK に存在しないため
// ---------------------------------------------------------------------------
static FbxAMatrix FbxMatrixToAMatrix(const FbxMatrix& m)
{
    FbxAMatrix am;
    am.SetRow(0, m.GetRow(0));
    am.SetRow(1, m.GetRow(1));
    am.SetRow(2, m.GetRow(2));
    am.SetRow(3, m.GetRow(3));
    return am;
}

static FbxMatrix FbxAMatrixToMatrix(const FbxAMatrix& am)
{
    FbxMatrix m;
    m.SetRow(0, am.GetRow(0));
    m.SetRow(1, am.GetRow(1));
    m.SetRow(2, am.GetRow(2));
    m.SetRow(3, am.GetRow(3));
    return m;
}

struct PoseEntry { FbxNode* node; FbxMatrix mat; bool isLocal; };

void GeometryProcessor::ProcessPoses(
    FbxScene* scene,
    const FbxAMatrix& convMatrix,
    const FbxAMatrix& convInv,
    double scale)
{
    int poseCount = scene->GetPoseCount();
    for (int pi = 0; pi < poseCount; ++pi)
    {
        FbxPose* pose = scene->GetPose(pi);
        if (!pose->IsBindPose()) continue;

        int entryCount = pose->GetCount();

        // 変換後の行列を退避してからエントリを再構築する
        std::vector<PoseEntry> entries;
        entries.reserve(entryCount);

        for (int ei = 0; ei < entryCount; ++ei)
        {
            FbxNode*  node    = pose->GetNode(ei);
            FbxMatrix rawMat  = pose->GetMatrix(ei);
            bool      isLocal = pose->IsLocalMatrix(ei);

            FbxAMatrix am    = FbxMatrixToAMatrix(rawMat);
            FbxAMatrix amNew = ConvertClusterMatrix(am, convMatrix, convInv, scale);
            entries.push_back({ node, FbxAMatrixToMatrix(amNew), isLocal });
        }

        // エントリを逆順に削除してから変換済みの行列で再追加
        for (int ei = entryCount - 1; ei >= 0; --ei)
            pose->Remove(ei);

        for (const PoseEntry& e : entries)
            pose->Add(e.node, e.mat, e.isLocal);

        std::cout << "[Geometry] Converted bind pose: " << pose->GetName()
                  << " (" << entryCount << " entries)\n";
    }
}

void GeometryProcessor::FlipWindingOrder(FbxMesh* mesh)
{
    int polyCount = mesh->GetPolygonCount();
    if (polyCount == 0) return;

    // Reverse polygon vertex indices
    // GetPolygonVertices() returns const int*, but the underlying array is mutable.
    // We use const_cast since we own the mesh data.
    int* polyVerts = const_cast<int*>(mesh->GetPolygonVertices());
    for (int pi = 0; pi < polyCount; ++pi)
    {
        int sz    = mesh->GetPolygonSize(pi);
        int start = mesh->GetPolygonVertexIndex(pi);
        if (sz < 3) continue;
        // Keep index [start+0], reverse [start+1 .. start+sz-1]
        std::reverse(polyVerts + start + 1, polyVerts + start + sz);
    }

    // Synchronize per-polygon-vertex layer elements
    int layerCount = mesh->GetLayerCount();
    for (int li = 0; li < layerCount; ++li)
    {
        FbxLayer* layer = mesh->GetLayer(li);
        if (!layer) continue;

        ReverseLayerElementByPolygon(layer->GetNormals(),   mesh);
        ReverseLayerElementByPolygon(layer->GetTangents(),  mesh);
        ReverseLayerElementByPolygon(layer->GetBinormals(), mesh);
        ReverseLayerElementByPolygon(layer->GetUVs(),       mesh);
    }
}
