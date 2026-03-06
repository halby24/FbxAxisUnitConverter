#include "GeometryProcessor.h"
#include <iostream>
#include <algorithm>

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
    int meshCount = scene->GetSrcObjectCount<FbxMesh>();
    for (int i = 0; i < meshCount; ++i)
    {
        FbxMesh* mesh = scene->GetSrcObject<FbxMesh>(i);
        ProcessMesh(mesh, convMatrix, scale);
    }
}

void GeometryProcessor::ProcessMesh(FbxMesh* mesh, const FbxAMatrix& convMatrix, double scale)
{
    if (mProcessedMeshes.count(mesh)) return;

    // --- Control points (vertex positions): rotate + scale ---
    int numVerts = mesh->GetControlPointsCount();
    FbxVector4* verts = mesh->GetControlPoints();
    for (int i = 0; i < numVerts; ++i)
    {
        FbxVector4 v(verts[i][0] * scale,
                     verts[i][1] * scale,
                     verts[i][2] * scale, 0.0);
        FbxVector4 vNew = convMatrix.MultT(v);
        verts[i] = FbxVector4(vNew[0], vNew[1], vNew[2], verts[i][3]);
    }

    // --- Layer elements: normals, tangents, binormals (rotate only, no scale) ---
    int layerCount = mesh->GetLayerCount();
    for (int li = 0; li < layerCount; ++li)
    {
        FbxLayer* layer = mesh->GetLayer(li);
        if (!layer) continue;

        // Normals
        if (auto* elem = layer->GetNormals())
        {
            if (elem->GetMappingMode() == FbxLayerElement::eByPolygonVertex ||
                elem->GetMappingMode() == FbxLayerElement::eByControlPoint)
            {
                if (elem->GetReferenceMode() == FbxLayerElement::eDirect)
                {
                    auto& arr = elem->GetDirectArray();
                    int cnt = arr.GetCount();
                    auto* ptr = static_cast<FbxVector4*>(arr.GetLocked(FbxLayerElementArray::eReadWriteLock));
                    if (ptr)
                    {
                        for (int i = 0; i < cnt; ++i)
                        {
                            FbxVector4& n = ptr[i];
                            FbxVector4 nNew = convMatrix.MultT(FbxVector4(n[0], n[1], n[2], 0.0));
                            n = FbxVector4(nNew[0], nNew[1], nNew[2], n[3]);
                        }
                        arr.Release(reinterpret_cast<void**>(&ptr));
                    }
                }
            }
        }

        // Tangents
        if (auto* elem = layer->GetTangents())
        {
            if (elem->GetReferenceMode() == FbxLayerElement::eDirect)
            {
                auto& arr = elem->GetDirectArray();
                int cnt = arr.GetCount();
                auto* ptr = static_cast<FbxVector4*>(arr.GetLocked(FbxLayerElementArray::eReadWriteLock));
                if (ptr)
                {
                    for (int i = 0; i < cnt; ++i)
                    {
                        FbxVector4& t = ptr[i];
                        FbxVector4 tNew = convMatrix.MultT(FbxVector4(t[0], t[1], t[2], 0.0));
                        t = FbxVector4(tNew[0], tNew[1], tNew[2], t[3]);
                    }
                    arr.Release(reinterpret_cast<void**>(&ptr));
                }
            }
        }

        // Binormals
        if (auto* elem = layer->GetBinormals())
        {
            if (elem->GetReferenceMode() == FbxLayerElement::eDirect)
            {
                auto& arr = elem->GetDirectArray();
                int cnt = arr.GetCount();
                auto* ptr = static_cast<FbxVector4*>(arr.GetLocked(FbxLayerElementArray::eReadWriteLock));
                if (ptr)
                {
                    for (int i = 0; i < cnt; ++i)
                    {
                        FbxVector4& b = ptr[i];
                        FbxVector4 bNew = convMatrix.MultT(FbxVector4(b[0], b[1], b[2], 0.0));
                        b = FbxVector4(bNew[0], bNew[1], bNew[2], b[3]);
                    }
                    arr.Release(reinterpret_cast<void**>(&ptr));
                }
            }
        }
    }

    // --- Winding order flip (when handedness changes) ---
    if (mFlipWinding)
        FlipWindingOrder(mesh);

    mProcessedMeshes.insert(mesh);
    std::cout << "[Geometry] Processed mesh: " << mesh->GetName() << "\n";
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
