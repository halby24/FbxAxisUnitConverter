// Test_NormalConversion.cpp
//
// 座標軸変換後にワールド座標系で見た法線が一致することを検証する。
//
// 原理:
//   変換前ワールド法線 = rotation(GlobalTransform_src) * mesh_normal_src
//   変換後ワールド法線 = rotation(GlobalTransform_dst) * mesh_normal_dst
//   期待: world_normal_dst == convMatrix * world_normal_src
//
// これにより GeometryProcessor の法線変換と TransformProcessor のノード変換が
// 整合していることを保証する。

#include "TestHelpers.h"
#include "GeometryProcessor.h"
#include "TransformProcessor.h"

// ---------------------------------------------------------------------------
// ヘルパー: メッシュに eByControlPoint / eDirect の法線を設定する
// ---------------------------------------------------------------------------
static void SetNormals(FbxMesh* mesh,
                       const FbxVector4* normals, int count)
{
    FbxLayer* layer = mesh->GetLayer(0);
    if (!layer)
    {
        mesh->CreateLayer();
        layer = mesh->GetLayer(0);
    }

    auto* elem = FbxLayerElementNormal::Create(mesh, "normals");
    elem->SetMappingMode(FbxLayerElement::eByControlPoint);
    elem->SetReferenceMode(FbxLayerElement::eDirect);
    for (int i = 0; i < count; ++i)
        elem->GetDirectArray().Add(normals[i]);
    layer->SetNormals(elem);
}

// ---------------------------------------------------------------------------
// ヘルパー: メッシュに eByPolygonVertex / eDirect の法線を設定する
// normals の要素数 = ポリゴン頂点数の合計（三角形1つなら 3）
// ---------------------------------------------------------------------------
static void SetNormalsByPolygonVertex(FbxMesh* mesh,
                                     const FbxVector4* normals, int count)
{
    FbxLayer* layer = mesh->GetLayer(0);
    if (!layer)
    {
        mesh->CreateLayer();
        layer = mesh->GetLayer(0);
    }

    auto* elem = FbxLayerElementNormal::Create(mesh, "normals");
    elem->SetMappingMode(FbxLayerElement::eByPolygonVertex);
    elem->SetReferenceMode(FbxLayerElement::eDirect);
    for (int i = 0; i < count; ++i)
        elem->GetDirectArray().Add(normals[i]);
    layer->SetNormals(elem);
}

// ---------------------------------------------------------------------------
// ヘルパー: メッシュに eByPolygonVertex / eIndexToDirect の法線を設定する
// uniqueNormals: 重複排除済み法線ベクトル（DirectArray に格納）
// indices: 各ポリゴン頂点が参照する DirectArray のインデックス
// ---------------------------------------------------------------------------
static void SetNormalsByPolygonVertexIndexed(FbxMesh* mesh,
                                            const FbxVector4* uniqueNormals, int uniqueCount,
                                            const int* indices, int indexCount)
{
    FbxLayer* layer = mesh->GetLayer(0);
    if (!layer)
    {
        mesh->CreateLayer();
        layer = mesh->GetLayer(0);
    }

    auto* elem = FbxLayerElementNormal::Create(mesh, "normals");
    elem->SetMappingMode(FbxLayerElement::eByPolygonVertex);
    elem->SetReferenceMode(FbxLayerElement::eIndexToDirect);
    for (int i = 0; i < uniqueCount; ++i)
        elem->GetDirectArray().Add(uniqueNormals[i]);
    for (int i = 0; i < indexCount; ++i)
        elem->GetIndexArray().Add(indices[i]);
    layer->SetNormals(elem);
}

// ---------------------------------------------------------------------------
// ヘルパー: メッシュから法線を取得する (eDirect / eIndexToDirect 両対応)
// ---------------------------------------------------------------------------
static FbxVector4 GetNormal(FbxMesh* mesh, int index)
{
    FbxLayer* layer = mesh->GetLayer(0);
    auto* elem = layer->GetNormals();
    if (elem->GetReferenceMode() == FbxLayerElement::eIndexToDirect)
    {
        int directIndex = elem->GetIndexArray().GetAt(index);
        return elem->GetDirectArray().GetAt(directIndex);
    }
    return elem->GetDirectArray().GetAt(index);
}

// ---------------------------------------------------------------------------
// ヘルパー: FbxAMatrix の回転部分(3x3)だけで FbxVector4 を変換する
// ---------------------------------------------------------------------------
static FbxVector4 RotateVector(const FbxAMatrix& m, const FbxVector4& v)
{
    // MultT は平行移動も含むので、3x3 回転部分だけ手動で適用
    double x = m.Get(0,0)*v[0] + m.Get(0,1)*v[1] + m.Get(0,2)*v[2];
    double y = m.Get(1,0)*v[0] + m.Get(1,1)*v[1] + m.Get(1,2)*v[2];
    double z = m.Get(2,0)*v[0] + m.Get(2,1)*v[1] + m.Get(2,2)*v[2];
    return FbxVector4(x, y, z, 0.0);
}

// ---------------------------------------------------------------------------
// ヘルパー: FbxVector4 の近似比較
// ---------------------------------------------------------------------------
static void ExpectVec4Near(const FbxVector4& a, const FbxVector4& b,
                           double tol = 1e-6, const char* msg = "")
{
    EXPECT_NEAR(a[0], b[0], tol) << msg << " [0]";
    EXPECT_NEAR(a[1], b[1], tol) << msg << " [1]";
    EXPECT_NEAR(a[2], b[2], tol) << msg << " [2]";
}

// ---------------------------------------------------------------------------
// ヘルパー: 任意の軸変換行列を構築する
//   src/dst の up, fwd, right AxisVector から convMatrix を作る
//   (FbxAxisUnitConverter.cpp の BuildMatrix と同じロジック)
// ---------------------------------------------------------------------------
static FbxAMatrix BuildConvMatrix(
    int srcUpAxis, int srcUpSign, int srcFwdAxis, int srcFwdSign,
    int dstUpAxis, int dstUpSign, int dstFwdAxis, int dstFwdSign)
{
    // right = cross(fwd, up)
    auto computeRight = [](int upAxis, int upSign, int fwdAxis, int fwdSign,
                           int& outAxis, int& outSign)
    {
        outAxis = 3 - upAxis - fwdAxis;
        int a = fwdAxis, b = upAxis;
        bool even = (a==0&&b==1) || (a==1&&b==2) || (a==2&&b==0);
        int permSign = even ? 1 : -1;
        outSign = fwdSign * upSign * permSign;
    };

    int srcRightAxis, srcRightSign;
    computeRight(srcUpAxis, srcUpSign, srcFwdAxis, srcFwdSign, srcRightAxis, srcRightSign);
    int dstRightAxis, dstRightSign;
    computeRight(dstUpAxis, dstUpSign, dstFwdAxis, dstFwdSign, dstRightAxis, dstRightSign);

    double m[3][3] = {};
    m[srcRightAxis][dstRightAxis] = (double)(srcRightSign * dstRightSign);
    m[srcUpAxis   ][dstUpAxis   ] = (double)(srcUpSign    * dstUpSign);
    m[srcFwdAxis  ][dstFwdAxis  ] = (double)(srcFwdSign   * dstFwdSign);

    FbxAMatrix mat;
    mat.SetRow(0, FbxVector4(m[0][0], m[0][1], m[0][2], 0.0));
    mat.SetRow(1, FbxVector4(m[1][0], m[1][1], m[1][2], 0.0));
    mat.SetRow(2, FbxVector4(m[2][0], m[2][1], m[2][2], 0.0));
    mat.SetRow(3, FbxVector4(0.0,     0.0,     0.0,     1.0));
    return mat;
}

// ===========================================================================
// Z-up → Y-up: 単純なケース（ノード変換なし）
// 法線 (0,0,1) は Z-up 系で「上向き」→ Y-up 系では (0,1,0) になるべき
// ===========================================================================
TEST(NormalConversion, ZupToYup_IdentityTransform)
{
    FbxTestContext ctx;

    FbxNode* node = ctx.AddNode("n");
    FbxMesh* mesh = ctx.AttachTriangle(node,
        FbxVector4(0,0,0), FbxVector4(1,0,0), FbxVector4(0,1,0));

    FbxVector4 normals[] = {
        FbxVector4(0, 0, 1, 0),
        FbxVector4(0, 0, 1, 0),
        FbxVector4(0, 0, 1, 0),
    };
    SetNormals(mesh, normals, 3);

    // 変換前のワールド法線を記録
    FbxAMatrix globalBefore = node->EvaluateGlobalTransform();
    FbxVector4 worldNormalBefore = RotateVector(globalBefore, GetNormal(mesh, 0));

    // 変換実行
    FbxAMatrix M = FbxTestContext::MakeZupToYupMatrix();
    GeometryProcessor geoProc(false);
    geoProc.ProcessScene(ctx.scene, M, 1.0);
    TransformProcessor::ProcessNode(ctx.scene->GetRootNode(), M, 1.0);

    // 変換後のワールド法線を取得
    FbxAMatrix globalAfter = node->EvaluateGlobalTransform();
    FbxVector4 worldNormalAfter = RotateVector(globalAfter, GetNormal(mesh, 0));

    // 期待値: convMatrix * worldNormalBefore == worldNormalAfter
    FbxVector4 expected = M.MultT(worldNormalBefore);
    ExpectVec4Near(worldNormalAfter, expected, 1e-6, "world normal mismatch");
}

// ===========================================================================
// Z-up → Y-up: ノードに回転あり
// ノードが Z 軸 45° 回転 → 法線もワールド空間で回転される
// ===========================================================================
TEST(NormalConversion, ZupToYup_RotatedNode)
{
    FbxTestContext ctx;

    FbxNode* node = ctx.AddNode("n",
        FbxDouble3(0, 0, 0),
        FbxDouble3(0, 0, 45),  // Z 軸 45° 回転
        FbxDouble3(1, 1, 1));
    FbxMesh* mesh = ctx.AttachTriangle(node);

    FbxVector4 normals[] = {
        FbxVector4(0, 0, 1, 0),
        FbxVector4(1, 0, 0, 0),
        FbxVector4(0, 1, 0, 0),
    };
    SetNormals(mesh, normals, 3);

    // 変換前のワールド法線を記録（3 頂点分）
    FbxAMatrix globalBefore = node->EvaluateGlobalTransform();
    FbxVector4 worldBefore[3];
    for (int i = 0; i < 3; ++i)
        worldBefore[i] = RotateVector(globalBefore, GetNormal(mesh, i));

    // 変換実行
    FbxAMatrix M = FbxTestContext::MakeZupToYupMatrix();
    GeometryProcessor geoProc(false);
    geoProc.ProcessScene(ctx.scene, M, 1.0);
    TransformProcessor::ProcessNode(ctx.scene->GetRootNode(), M, 1.0);

    // 変換後のワールド法線
    FbxAMatrix globalAfter = node->EvaluateGlobalTransform();
    for (int i = 0; i < 3; ++i)
    {
        FbxVector4 worldAfter = RotateVector(globalAfter, GetNormal(mesh, i));
        FbxVector4 expected = M.MultT(worldBefore[i]);
        ExpectVec4Near(worldAfter, expected, 1e-6,
            ("normal[" + std::to_string(i) + "]").c_str());
    }
}

// ===========================================================================
// Z-up → Y-up: 階層構造（親回転 + 子回転）
// ===========================================================================
TEST(NormalConversion, ZupToYup_Hierarchy)
{
    FbxTestContext ctx;

    FbxNode* parent = ctx.AddNode("parent",
        FbxDouble3(10, 0, 0),
        FbxDouble3(0, 0, 30),  // Z 軸 30° 回転
        FbxDouble3(1, 1, 1));

    FbxNode* child = ctx.AddChildNode(parent, "child",
        FbxDouble3(5, 0, 0),
        FbxDouble3(0, 0, 60),  // Z 軸 60° 回転
        FbxDouble3(1, 1, 1));

    FbxMesh* mesh = ctx.AttachTriangle(child);
    FbxVector4 normals[] = {
        FbxVector4(0, 0, 1, 0),
        FbxVector4(1, 0, 0, 0),
        FbxVector4(0, 1, 0, 0),
    };
    SetNormals(mesh, normals, 3);

    // 変換前: 子ノードのグローバル変換でワールド法線を計算
    FbxAMatrix globalBefore = child->EvaluateGlobalTransform();
    FbxVector4 worldBefore[3];
    for (int i = 0; i < 3; ++i)
        worldBefore[i] = RotateVector(globalBefore, GetNormal(mesh, i));

    // 変換実行
    FbxAMatrix M = FbxTestContext::MakeZupToYupMatrix();
    GeometryProcessor geoProc(false);
    geoProc.ProcessScene(ctx.scene, M, 1.0);
    TransformProcessor::ProcessNode(ctx.scene->GetRootNode(), M, 1.0);

    // 変換後
    FbxAMatrix globalAfter = child->EvaluateGlobalTransform();
    for (int i = 0; i < 3; ++i)
    {
        FbxVector4 worldAfter = RotateVector(globalAfter, GetNormal(mesh, i));
        FbxVector4 expected = M.MultT(worldBefore[i]);
        ExpectVec4Near(worldAfter, expected, 1e-4,
            ("hierarchy normal[" + std::to_string(i) + "]").c_str());
    }
}

// ===========================================================================
// Y-up → Z-up (逆方向): 法線の整合性確認
// ===========================================================================
TEST(NormalConversion, YupToZup_RotatedNode)
{
    FbxTestContext ctx;

    // Y-up(-Z forward) → Z-up(+Y forward) の変換行列
    // src: up=Y(+1), fwd=Z(-1)  dst: up=Z(+1), fwd=Y(+1)
    FbxAMatrix M = BuildConvMatrix(
        1, +1, 2, -1,   // src: Y-up, -Z forward
        2, +1, 1, +1);  // dst: Z-up, +Y forward

    FbxNode* node = ctx.AddNode("n",
        FbxDouble3(3, 4, 5),
        FbxDouble3(30, 45, 0),
        FbxDouble3(1, 1, 1));
    FbxMesh* mesh = ctx.AttachTriangle(node);

    FbxVector4 normals[] = {
        FbxVector4(0, 1, 0, 0),  // Y-up 系での「上向き」
        FbxVector4(0, 0, -1, 0), // Y-up 系での「奥向き」
        FbxVector4(1, 0, 0, 0),
    };
    SetNormals(mesh, normals, 3);

    FbxAMatrix globalBefore = node->EvaluateGlobalTransform();
    FbxVector4 worldBefore[3];
    for (int i = 0; i < 3; ++i)
        worldBefore[i] = RotateVector(globalBefore, GetNormal(mesh, i));

    GeometryProcessor geoProc(false);
    geoProc.ProcessScene(ctx.scene, M, 1.0);
    TransformProcessor::ProcessNode(ctx.scene->GetRootNode(), M, 1.0);

    FbxAMatrix globalAfter = node->EvaluateGlobalTransform();
    for (int i = 0; i < 3; ++i)
    {
        FbxVector4 worldAfter = RotateVector(globalAfter, GetNormal(mesh, i));
        FbxVector4 expected = M.MultT(worldBefore[i]);
        ExpectVec4Near(worldAfter, expected, 1e-4,
            ("Y->Z normal[" + std::to_string(i) + "]").c_str());
    }
}

// ===========================================================================
// 斜め法線: 正規化されていない方向ベクトルでも整合性が保たれるか
// ===========================================================================
TEST(NormalConversion, ZupToYup_DiagonalNormals)
{
    FbxTestContext ctx;

    FbxNode* node = ctx.AddNode("n",
        FbxDouble3(0, 0, 0),
        FbxDouble3(0, 0, 0),
        FbxDouble3(1, 1, 1));
    FbxMesh* mesh = ctx.AttachTriangle(node);

    // 斜め方向の法線 (正規化済み)
    double inv = 1.0 / std::sqrt(3.0);
    FbxVector4 normals[] = {
        FbxVector4( inv,  inv,  inv, 0),
        FbxVector4(-inv,  inv,  inv, 0),
        FbxVector4( inv, -inv,  inv, 0),
    };
    SetNormals(mesh, normals, 3);

    FbxAMatrix globalBefore = node->EvaluateGlobalTransform();
    FbxVector4 worldBefore[3];
    for (int i = 0; i < 3; ++i)
        worldBefore[i] = RotateVector(globalBefore, GetNormal(mesh, i));

    FbxAMatrix M = FbxTestContext::MakeZupToYupMatrix();
    GeometryProcessor geoProc(false);
    geoProc.ProcessScene(ctx.scene, M, 1.0);
    TransformProcessor::ProcessNode(ctx.scene->GetRootNode(), M, 1.0);

    FbxAMatrix globalAfter = node->EvaluateGlobalTransform();
    for (int i = 0; i < 3; ++i)
    {
        FbxVector4 worldAfter = RotateVector(globalAfter, GetNormal(mesh, i));
        FbxVector4 expected = M.MultT(worldBefore[i]);
        ExpectVec4Near(worldAfter, expected, 1e-6,
            ("diagonal normal[" + std::to_string(i) + "]").c_str());
    }
}

// ===========================================================================
// 単位変換あり: スケールは法線に影響しない（方向のみ）ことを確認
// ===========================================================================
TEST(NormalConversion, ZupToYup_WithUnitScale)
{
    FbxTestContext ctx;

    FbxNode* node = ctx.AddNode("n",
        FbxDouble3(100, 0, 0),
        FbxDouble3(0, 0, 45),
        FbxDouble3(1, 1, 1));
    FbxMesh* mesh = ctx.AttachTriangle(node);

    FbxVector4 normals[] = {
        FbxVector4(0, 0, 1, 0),
        FbxVector4(1, 0, 0, 0),
        FbxVector4(0, 1, 0, 0),
    };
    SetNormals(mesh, normals, 3);

    FbxAMatrix globalBefore = node->EvaluateGlobalTransform();
    FbxVector4 worldBefore[3];
    for (int i = 0; i < 3; ++i)
        worldBefore[i] = RotateVector(globalBefore, GetNormal(mesh, i));

    // スケール 0.01 (cm→m)
    FbxAMatrix M = FbxTestContext::MakeZupToYupMatrix();
    double scale = 0.01;
    GeometryProcessor geoProc(false);
    geoProc.ProcessScene(ctx.scene, M, scale);
    TransformProcessor::ProcessNode(ctx.scene->GetRootNode(), M, scale);

    // 変換後: EvaluateGlobalTransform は Translation にスケールが反映されるが
    // 法線はスケールの影響を受けない
    FbxAMatrix globalAfter = node->EvaluateGlobalTransform();
    for (int i = 0; i < 3; ++i)
    {
        FbxVector4 worldAfter = RotateVector(globalAfter, GetNormal(mesh, i));
        FbxVector4 expected = M.MultT(worldBefore[i]);
        ExpectVec4Near(worldAfter, expected, 1e-4,
            ("scaled normal[" + std::to_string(i) + "]").c_str());
    }
}

// ===========================================================================
// X-up → Y-up: 非標準的な軸変換
// ===========================================================================
TEST(NormalConversion, XupToYup_RotatedNode)
{
    FbxTestContext ctx;

    // X-up(+Z forward) → Y-up(-Z forward)
    FbxAMatrix M = BuildConvMatrix(
        0, +1, 2, +1,   // src: X-up, +Z forward
        1, +1, 2, -1);  // dst: Y-up, -Z forward

    FbxNode* node = ctx.AddNode("n",
        FbxDouble3(1, 2, 3),
        FbxDouble3(15, 30, 45),
        FbxDouble3(1, 1, 1));
    FbxMesh* mesh = ctx.AttachTriangle(node);

    FbxVector4 normals[] = {
        FbxVector4(1, 0, 0, 0),  // X-up 系での「上向き」
        FbxVector4(0, 0, 1, 0),
        FbxVector4(0, 1, 0, 0),
    };
    SetNormals(mesh, normals, 3);

    FbxAMatrix globalBefore = node->EvaluateGlobalTransform();
    FbxVector4 worldBefore[3];
    for (int i = 0; i < 3; ++i)
        worldBefore[i] = RotateVector(globalBefore, GetNormal(mesh, i));

    GeometryProcessor geoProc(false);
    geoProc.ProcessScene(ctx.scene, M, 1.0);
    TransformProcessor::ProcessNode(ctx.scene->GetRootNode(), M, 1.0);

    FbxAMatrix globalAfter = node->EvaluateGlobalTransform();
    for (int i = 0; i < 3; ++i)
    {
        FbxVector4 worldAfter = RotateVector(globalAfter, GetNormal(mesh, i));
        FbxVector4 expected = M.MultT(worldBefore[i]);
        ExpectVec4Near(worldAfter, expected, 1e-4,
            ("X->Y normal[" + std::to_string(i) + "]").c_str());
    }
}

// ===========================================================================
// eByPolygonVertex テスト群
// ===========================================================================

// ---------------------------------------------------------------------------
// ByPolygonVertex: Z-up → Y-up、ノード変換なし
// ---------------------------------------------------------------------------
TEST(NormalConversion, ByPolygonVertex_ZupToYup_IdentityTransform)
{
    FbxTestContext ctx;

    FbxNode* node = ctx.AddNode("n");
    FbxMesh* mesh = ctx.AttachTriangle(node,
        FbxVector4(0,0,0), FbxVector4(1,0,0), FbxVector4(0,1,0));

    // eByPolygonVertex: ポリゴン頂点ごとに法線を持つ（三角形1つ → 3つ）
    FbxVector4 normals[] = {
        FbxVector4(0, 0, 1, 0),
        FbxVector4(0, 0, 1, 0),
        FbxVector4(0, 0, 1, 0),
    };
    SetNormalsByPolygonVertex(mesh, normals, 3);

    FbxAMatrix globalBefore = node->EvaluateGlobalTransform();
    FbxVector4 worldBefore[3];
    for (int i = 0; i < 3; ++i)
        worldBefore[i] = RotateVector(globalBefore, GetNormal(mesh, i));

    FbxAMatrix M = FbxTestContext::MakeZupToYupMatrix();
    GeometryProcessor geoProc(false);
    geoProc.ProcessScene(ctx.scene, M, 1.0);
    TransformProcessor::ProcessNode(ctx.scene->GetRootNode(), M, 1.0);

    FbxAMatrix globalAfter = node->EvaluateGlobalTransform();
    for (int i = 0; i < 3; ++i)
    {
        FbxVector4 worldAfter = RotateVector(globalAfter, GetNormal(mesh, i));
        FbxVector4 expected = M.MultT(worldBefore[i]);
        ExpectVec4Near(worldAfter, expected, 1e-6,
            ("bpv identity normal[" + std::to_string(i) + "]").c_str());
    }
}

// ---------------------------------------------------------------------------
// ByPolygonVertex: Z-up → Y-up、ノードに回転あり
// ---------------------------------------------------------------------------
TEST(NormalConversion, ByPolygonVertex_ZupToYup_RotatedNode)
{
    FbxTestContext ctx;

    FbxNode* node = ctx.AddNode("n",
        FbxDouble3(0, 0, 0),
        FbxDouble3(0, 0, 45),
        FbxDouble3(1, 1, 1));
    FbxMesh* mesh = ctx.AttachTriangle(node);

    FbxVector4 normals[] = {
        FbxVector4(0, 0, 1, 0),
        FbxVector4(1, 0, 0, 0),
        FbxVector4(0, 1, 0, 0),
    };
    SetNormalsByPolygonVertex(mesh, normals, 3);

    FbxAMatrix globalBefore = node->EvaluateGlobalTransform();
    FbxVector4 worldBefore[3];
    for (int i = 0; i < 3; ++i)
        worldBefore[i] = RotateVector(globalBefore, GetNormal(mesh, i));

    FbxAMatrix M = FbxTestContext::MakeZupToYupMatrix();
    GeometryProcessor geoProc(false);
    geoProc.ProcessScene(ctx.scene, M, 1.0);
    TransformProcessor::ProcessNode(ctx.scene->GetRootNode(), M, 1.0);

    FbxAMatrix globalAfter = node->EvaluateGlobalTransform();
    for (int i = 0; i < 3; ++i)
    {
        FbxVector4 worldAfter = RotateVector(globalAfter, GetNormal(mesh, i));
        FbxVector4 expected = M.MultT(worldBefore[i]);
        ExpectVec4Near(worldAfter, expected, 1e-6,
            ("bpv rotated normal[" + std::to_string(i) + "]").c_str());
    }
}

// ---------------------------------------------------------------------------
// ByPolygonVertex: 階層構造（親+子回転）
// ---------------------------------------------------------------------------
TEST(NormalConversion, ByPolygonVertex_ZupToYup_Hierarchy)
{
    FbxTestContext ctx;

    FbxNode* parent = ctx.AddNode("parent",
        FbxDouble3(10, 0, 0),
        FbxDouble3(0, 0, 30),
        FbxDouble3(1, 1, 1));

    FbxNode* child = ctx.AddChildNode(parent, "child",
        FbxDouble3(5, 0, 0),
        FbxDouble3(0, 0, 60),
        FbxDouble3(1, 1, 1));

    FbxMesh* mesh = ctx.AttachTriangle(child);
    FbxVector4 normals[] = {
        FbxVector4(0, 0, 1, 0),
        FbxVector4(1, 0, 0, 0),
        FbxVector4(0, 1, 0, 0),
    };
    SetNormalsByPolygonVertex(mesh, normals, 3);

    FbxAMatrix globalBefore = child->EvaluateGlobalTransform();
    FbxVector4 worldBefore[3];
    for (int i = 0; i < 3; ++i)
        worldBefore[i] = RotateVector(globalBefore, GetNormal(mesh, i));

    FbxAMatrix M = FbxTestContext::MakeZupToYupMatrix();
    GeometryProcessor geoProc(false);
    geoProc.ProcessScene(ctx.scene, M, 1.0);
    TransformProcessor::ProcessNode(ctx.scene->GetRootNode(), M, 1.0);

    FbxAMatrix globalAfter = child->EvaluateGlobalTransform();
    for (int i = 0; i < 3; ++i)
    {
        FbxVector4 worldAfter = RotateVector(globalAfter, GetNormal(mesh, i));
        FbxVector4 expected = M.MultT(worldBefore[i]);
        ExpectVec4Near(worldAfter, expected, 1e-4,
            ("bpv hierarchy normal[" + std::to_string(i) + "]").c_str());
    }
}

// ---------------------------------------------------------------------------
// ByPolygonVertex: Y-up → Z-up (逆方向)
// ---------------------------------------------------------------------------
TEST(NormalConversion, ByPolygonVertex_YupToZup_RotatedNode)
{
    FbxTestContext ctx;

    FbxAMatrix M = BuildConvMatrix(
        1, +1, 2, -1,
        2, +1, 1, +1);

    FbxNode* node = ctx.AddNode("n",
        FbxDouble3(3, 4, 5),
        FbxDouble3(30, 45, 0),
        FbxDouble3(1, 1, 1));
    FbxMesh* mesh = ctx.AttachTriangle(node);

    FbxVector4 normals[] = {
        FbxVector4(0, 1, 0, 0),
        FbxVector4(0, 0, -1, 0),
        FbxVector4(1, 0, 0, 0),
    };
    SetNormalsByPolygonVertex(mesh, normals, 3);

    FbxAMatrix globalBefore = node->EvaluateGlobalTransform();
    FbxVector4 worldBefore[3];
    for (int i = 0; i < 3; ++i)
        worldBefore[i] = RotateVector(globalBefore, GetNormal(mesh, i));

    GeometryProcessor geoProc(false);
    geoProc.ProcessScene(ctx.scene, M, 1.0);
    TransformProcessor::ProcessNode(ctx.scene->GetRootNode(), M, 1.0);

    FbxAMatrix globalAfter = node->EvaluateGlobalTransform();
    for (int i = 0; i < 3; ++i)
    {
        FbxVector4 worldAfter = RotateVector(globalAfter, GetNormal(mesh, i));
        FbxVector4 expected = M.MultT(worldBefore[i]);
        ExpectVec4Near(worldAfter, expected, 1e-4,
            ("bpv Y->Z normal[" + std::to_string(i) + "]").c_str());
    }
}

// ---------------------------------------------------------------------------
// ByPolygonVertex: 頂点ごとに異なる法線方向（スムーズシェーディング的）
// 同一コントロールポイントでもポリゴンごとに法線が異なるケースを検証
// ---------------------------------------------------------------------------
TEST(NormalConversion, ByPolygonVertex_DifferentNormalsPerVertex)
{
    FbxTestContext ctx;

    // 2 三角形が頂点 v1 を共有するクアッド風メッシュ
    // v2---v3
    // | \ |
    // v0---v1
    FbxNode* node = ctx.AddNode("n",
        FbxDouble3(0, 0, 0),
        FbxDouble3(0, 0, 30),
        FbxDouble3(1, 1, 1));

    FbxMesh* mesh = FbxMesh::Create(ctx.scene, "quad");
    mesh->InitControlPoints(4);
    mesh->SetControlPointAt(FbxVector4(0, 0, 0), 0);
    mesh->SetControlPointAt(FbxVector4(1, 0, 0), 1);
    mesh->SetControlPointAt(FbxVector4(0, 1, 0), 2);
    mesh->SetControlPointAt(FbxVector4(1, 1, 0), 3);

    // ポリゴン 0: v0, v1, v2
    mesh->BeginPolygon();
    mesh->AddPolygon(0);
    mesh->AddPolygon(1);
    mesh->AddPolygon(2);
    mesh->EndPolygon();

    // ポリゴン 1: v1, v3, v2
    mesh->BeginPolygon();
    mesh->AddPolygon(1);
    mesh->AddPolygon(3);
    mesh->AddPolygon(2);
    mesh->EndPolygon();

    node->SetNodeAttribute(mesh);

    // 6 法線（ポリゴン頂点ごと）: 同じ頂点でもポリゴンが違えば法線が異なる
    double inv = 1.0 / std::sqrt(2.0);
    FbxVector4 normals[] = {
        // poly0: v0, v1, v2
        FbxVector4(0, 0, 1, 0),
        FbxVector4(inv, 0, inv, 0),   // v1 at poly0
        FbxVector4(0, 0, 1, 0),
        // poly1: v1, v3, v2
        FbxVector4(-inv, 0, inv, 0),  // v1 at poly1 (異なる法線!)
        FbxVector4(0, inv, inv, 0),
        FbxVector4(0, 0, 1, 0),
    };
    SetNormalsByPolygonVertex(mesh, normals, 6);

    FbxAMatrix globalBefore = node->EvaluateGlobalTransform();
    FbxVector4 worldBefore[6];
    for (int i = 0; i < 6; ++i)
        worldBefore[i] = RotateVector(globalBefore, GetNormal(mesh, i));

    FbxAMatrix M = FbxTestContext::MakeZupToYupMatrix();
    GeometryProcessor geoProc(false);
    geoProc.ProcessScene(ctx.scene, M, 1.0);
    TransformProcessor::ProcessNode(ctx.scene->GetRootNode(), M, 1.0);

    FbxAMatrix globalAfter = node->EvaluateGlobalTransform();
    for (int i = 0; i < 6; ++i)
    {
        FbxVector4 worldAfter = RotateVector(globalAfter, GetNormal(mesh, i));
        FbxVector4 expected = M.MultT(worldBefore[i]);
        ExpectVec4Near(worldAfter, expected, 1e-6,
            ("bpv multi-poly normal[" + std::to_string(i) + "]").c_str());
    }
}

// ===========================================================================
// eIndexToDirect テスト群
// 今回のバグの再現ケース: eIndexToDirect 法線が変換されず 90° ズレる問題
// ===========================================================================

// ---------------------------------------------------------------------------
// IndexToDirect: Z-up → Y-up、ノード変換なし
// ---------------------------------------------------------------------------
TEST(NormalConversion, IndexToDirect_ZupToYup_IdentityTransform)
{
    FbxTestContext ctx;

    FbxNode* node = ctx.AddNode("n");
    FbxMesh* mesh = ctx.AttachTriangle(node,
        FbxVector4(0,0,0), FbxVector4(1,0,0), FbxVector4(0,1,0));

    // DirectArray に1つだけ法線を入れ、3頂点すべてがそれを参照
    FbxVector4 uniqueNormals[] = { FbxVector4(0, 0, 1, 0) };
    int indices[] = { 0, 0, 0 };
    SetNormalsByPolygonVertexIndexed(mesh, uniqueNormals, 1, indices, 3);

    FbxAMatrix globalBefore = node->EvaluateGlobalTransform();
    FbxVector4 worldBefore = RotateVector(globalBefore, GetNormal(mesh, 0));

    FbxAMatrix M = FbxTestContext::MakeZupToYupMatrix();
    GeometryProcessor geoProc(false);
    geoProc.ProcessScene(ctx.scene, M, 1.0);
    TransformProcessor::ProcessNode(ctx.scene->GetRootNode(), M, 1.0);

    FbxAMatrix globalAfter = node->EvaluateGlobalTransform();
    FbxVector4 worldAfter = RotateVector(globalAfter, GetNormal(mesh, 0));
    FbxVector4 expected = M.MultT(worldBefore);
    ExpectVec4Near(worldAfter, expected, 1e-6, "idx2dir identity");
}

// ---------------------------------------------------------------------------
// IndexToDirect: Z-up → Y-up、ノードに回転あり + 複数ユニーク法線
// ---------------------------------------------------------------------------
TEST(NormalConversion, IndexToDirect_ZupToYup_RotatedNode)
{
    FbxTestContext ctx;

    FbxNode* node = ctx.AddNode("n",
        FbxDouble3(0, 0, 0),
        FbxDouble3(0, 0, 45),
        FbxDouble3(1, 1, 1));
    FbxMesh* mesh = ctx.AttachTriangle(node);

    // 2 つのユニーク法線を 3 頂点に割り当て
    FbxVector4 uniqueNormals[] = {
        FbxVector4(0, 0, 1, 0),
        FbxVector4(1, 0, 0, 0),
    };
    int indices[] = { 0, 1, 0 };
    SetNormalsByPolygonVertexIndexed(mesh, uniqueNormals, 2, indices, 3);

    FbxAMatrix globalBefore = node->EvaluateGlobalTransform();
    FbxVector4 worldBefore[3];
    for (int i = 0; i < 3; ++i)
        worldBefore[i] = RotateVector(globalBefore, GetNormal(mesh, i));

    FbxAMatrix M = FbxTestContext::MakeZupToYupMatrix();
    GeometryProcessor geoProc(false);
    geoProc.ProcessScene(ctx.scene, M, 1.0);
    TransformProcessor::ProcessNode(ctx.scene->GetRootNode(), M, 1.0);

    FbxAMatrix globalAfter = node->EvaluateGlobalTransform();
    for (int i = 0; i < 3; ++i)
    {
        FbxVector4 worldAfter = RotateVector(globalAfter, GetNormal(mesh, i));
        FbxVector4 expected = M.MultT(worldBefore[i]);
        ExpectVec4Near(worldAfter, expected, 1e-6,
            ("idx2dir rotated normal[" + std::to_string(i) + "]").c_str());
    }
}

// ---------------------------------------------------------------------------
// IndexToDirect: 複数ポリゴン + 共有法線（Icosphere 的パターン）
// DirectArray に少数のユニーク法線、IndexArray で多数の頂点が参照
// ---------------------------------------------------------------------------
TEST(NormalConversion, IndexToDirect_SharedNormals_MultiPolygon)
{
    FbxTestContext ctx;

    FbxNode* node = ctx.AddNode("n",
        FbxDouble3(0, 0, 0),
        FbxDouble3(30, 0, 0),
        FbxDouble3(1, 1, 1));

    // 4 頂点、2 三角形のクアッド
    FbxMesh* mesh = FbxMesh::Create(ctx.scene, "quad");
    mesh->InitControlPoints(4);
    mesh->SetControlPointAt(FbxVector4(0, 0, 0), 0);
    mesh->SetControlPointAt(FbxVector4(1, 0, 0), 1);
    mesh->SetControlPointAt(FbxVector4(0, 1, 0), 2);
    mesh->SetControlPointAt(FbxVector4(1, 1, 0), 3);

    mesh->BeginPolygon();
    mesh->AddPolygon(0); mesh->AddPolygon(1); mesh->AddPolygon(2);
    mesh->EndPolygon();
    mesh->BeginPolygon();
    mesh->AddPolygon(1); mesh->AddPolygon(3); mesh->AddPolygon(2);
    mesh->EndPolygon();
    node->SetNodeAttribute(mesh);

    // 3 ユニーク法線、6 ポリゴン頂点が参照
    double inv = 1.0 / std::sqrt(2.0);
    FbxVector4 uniqueNormals[] = {
        FbxVector4(0, 0, 1, 0),
        FbxVector4(inv, 0, inv, 0),
        FbxVector4(0, inv, inv, 0),
    };
    int indices[] = { 0, 1, 0, 1, 2, 0 };
    SetNormalsByPolygonVertexIndexed(mesh, uniqueNormals, 3, indices, 6);

    FbxAMatrix globalBefore = node->EvaluateGlobalTransform();
    FbxVector4 worldBefore[6];
    for (int i = 0; i < 6; ++i)
        worldBefore[i] = RotateVector(globalBefore, GetNormal(mesh, i));

    FbxAMatrix M = FbxTestContext::MakeZupToYupMatrix();
    GeometryProcessor geoProc(false);
    geoProc.ProcessScene(ctx.scene, M, 1.0);
    TransformProcessor::ProcessNode(ctx.scene->GetRootNode(), M, 1.0);

    FbxAMatrix globalAfter = node->EvaluateGlobalTransform();
    for (int i = 0; i < 6; ++i)
    {
        FbxVector4 worldAfter = RotateVector(globalAfter, GetNormal(mesh, i));
        FbxVector4 expected = M.MultT(worldBefore[i]);
        ExpectVec4Near(worldAfter, expected, 1e-4,
            ("idx2dir shared normal[" + std::to_string(i) + "]").c_str());
    }
}

// ---------------------------------------------------------------------------
// IndexToDirect: Y-up → Z-up (逆方向)
// ---------------------------------------------------------------------------
TEST(NormalConversion, IndexToDirect_YupToZup)
{
    FbxTestContext ctx;

    FbxAMatrix M = BuildConvMatrix(
        1, +1, 2, -1,
        2, +1, 1, +1);

    FbxNode* node = ctx.AddNode("n",
        FbxDouble3(3, 4, 5),
        FbxDouble3(30, 45, 0),
        FbxDouble3(1, 1, 1));
    FbxMesh* mesh = ctx.AttachTriangle(node);

    FbxVector4 uniqueNormals[] = {
        FbxVector4(0, 1, 0, 0),
        FbxVector4(0, 0, -1, 0),
        FbxVector4(1, 0, 0, 0),
    };
    int indices[] = { 0, 1, 2 };
    SetNormalsByPolygonVertexIndexed(mesh, uniqueNormals, 3, indices, 3);

    FbxAMatrix globalBefore = node->EvaluateGlobalTransform();
    FbxVector4 worldBefore[3];
    for (int i = 0; i < 3; ++i)
        worldBefore[i] = RotateVector(globalBefore, GetNormal(mesh, i));

    GeometryProcessor geoProc(false);
    geoProc.ProcessScene(ctx.scene, M, 1.0);
    TransformProcessor::ProcessNode(ctx.scene->GetRootNode(), M, 1.0);

    FbxAMatrix globalAfter = node->EvaluateGlobalTransform();
    for (int i = 0; i < 3; ++i)
    {
        FbxVector4 worldAfter = RotateVector(globalAfter, GetNormal(mesh, i));
        FbxVector4 expected = M.MultT(worldBefore[i]);
        ExpectVec4Near(worldAfter, expected, 1e-4,
            ("idx2dir Y->Z normal[" + std::to_string(i) + "]").c_str());
    }
}
