#include "FbxAxisUnitConverter.h"
#include "GeometryProcessor.h"
#include "TransformProcessor.h"
#include "AnimProcessor.h"
#include <iostream>
#include <stdexcept>
#include <cmath>

// ---------------------------------------------------------------------------
// Helper: compute right = cross(fwd_into_screen, up) for right-handed system
// ---------------------------------------------------------------------------
static AxisVector ComputeRight(const AxisVector& up, const AxisVector& fwd)
{
    // The remaining axis (0+1+2 = 3)
    int rightAxis = 3 - up.axis - fwd.axis;

    // cross(fwd, up): sign depends on cyclic parity of (fwd.axis, up.axis, rightAxis)
    // Even permutation of (0,1,2): (0,1,2), (1,2,0), (2,0,1) -> permSign = +1
    // Odd permutation: (0,2,1), (1,0,2), (2,1,0) -> permSign = -1
    int a = fwd.axis, b = up.axis;
    bool even = (a == 0 && b == 1) || (a == 1 && b == 2) || (a == 2 && b == 0);
    int permSign = even ? 1 : -1;
    int rightSign = fwd.sign * up.sign * permSign;

    return AxisVector{ rightAxis, rightSign };
}

// ---------------------------------------------------------------------------
// Helper: decode FbxAxisSystem -> (up, fwd_into_screen) AxisVectors
//
// FbxAxisSystem "front vector" is TOWARD the viewer (out of screen).
// We store "fwd" as INTO the screen, so fwd.sign = -frontSign.
//
// Parity rule (from SDK header comment):
//   up=X: ParityEven=Y, ParityOdd=Z
//   up=Y: ParityEven=X, ParityOdd=Z
//   up=Z: ParityEven=X, ParityOdd=Y
// ---------------------------------------------------------------------------
static std::pair<AxisVector, AxisVector> DecodeAxisSystem(const FbxAxisSystem& sys)
{
    int upSign = 1, frontSign = 1;
    FbxAxisSystem::EUpVector    upVec   = sys.GetUpVector(upSign);
    FbxAxisSystem::EFrontVector frontParity = sys.GetFrontVector(frontSign);

    AxisVector up;
    up.axis = (int)upVec - 1;   // eXAxis=1->0, eYAxis=2->1, eZAxis=3->2
    up.sign = upSign;

    // Find front axis from parity and up axis
    // "min of remaining" = ParityEven, "max of remaining" = ParityOdd
    int upIdx = up.axis;
    int minRemaining = (upIdx == 0) ? 1 : 0;
    int maxRemaining = (upIdx == 2) ? 1 : 2;
    int frontAxis = (frontParity == FbxAxisSystem::eParityEven) ? minRemaining : maxRemaining;

    AxisVector fwd;
    fwd.axis = frontAxis;
    // FBX frontSign=+1 means toward viewer; our fwd is into screen -> negate
    fwd.sign = -frontSign;

    return { up, fwd };
}

// ---------------------------------------------------------------------------
// Helper: build FbxAxisSystem from (up, fwd_into_screen) AxisVectors
// Always constructs a right-handed system (left-handed is not supported).
// ---------------------------------------------------------------------------
static FbxAxisSystem MakeAxisSystem(const AxisVector& up, const AxisVector& fwd)
{
    // FBX EUpVector is 1-indexed
    FbxAxisSystem::EUpVector upVec = (FbxAxisSystem::EUpVector)(up.axis + 1);

    // Map fwd axis to parity
    int upIdx = up.axis;
    int minRemaining = (upIdx == 0) ? 1 : 0;
    int maxRemaining = (upIdx == 2) ? 1 : 2;
    FbxAxisSystem::EFrontVector parity =
        (fwd.axis == minRemaining) ? FbxAxisSystem::eParityEven : FbxAxisSystem::eParityOdd;

    // FBX front is toward viewer = opposite of our into-screen fwd
    int fbxFrontSign = -fwd.sign;

    // Determine handedness by checking right vector sign
    // right = cross(fwd, up); if rightAxis naturally gets positive sign -> right-handed
    AxisVector right = ComputeRight(up, fwd);
    // The "natural" right-hand right vector is what we computed.
    // If right.sign > 0, the system is right-handed in the traditional sense.
    // We always output right-handed here.
    FbxAxisSystem::ECoordSystem coordSys = FbxAxisSystem::eRightHanded;

    // Constructor: (upSign * EUpVector, frontSign * EFrontVector, ECoordSystem)
    return FbxAxisSystem(
        (FbxAxisSystem::EUpVector)(up.sign * (int)upVec),
        (FbxAxisSystem::EFrontVector)(fbxFrontSign * (int)parity),
        coordSys
    );
}

// ---------------------------------------------------------------------------
// Helper: build 4x4 affine conversion matrix from src/dst axis vectors
// Uses column-vector convention: v' = M * v  (as per FbxAMatrix::MultT)
// M[dstRole.axis][srcRole.axis] = dstRole.sign * srcRole.sign
// ---------------------------------------------------------------------------
static void BuildMatrix(
    const AxisVector& srcUp, const AxisVector& srcFwd, const AxisVector& srcRight,
    const AxisVector& dstUp, const AxisVector& dstFwd, const AxisVector& dstRight,
    FbxAMatrix& outMatrix, bool& outFlipWinding)
{
    double m[3][3] = {};
    m[srcRight.axis][dstRight.axis] = (double)(srcRight.sign * dstRight.sign);
    m[srcUp.axis   ][dstUp.axis   ] = (double)(srcUp.sign    * dstUp.sign   );
    m[srcFwd.axis  ][dstFwd.axis  ] = (double)(srcFwd.sign   * dstFwd.sign  );

    // Set rows of the 4x4 matrix (row 3 = translation, w = 1)
    outMatrix.SetRow(0, FbxVector4(m[0][0], m[0][1], m[0][2], 0.0));
    outMatrix.SetRow(1, FbxVector4(m[1][0], m[1][1], m[1][2], 0.0));
    outMatrix.SetRow(2, FbxVector4(m[2][0], m[2][1], m[2][2], 0.0));
    outMatrix.SetRow(3, FbxVector4(0.0,     0.0,     0.0,     1.0));

    // Determinant of 3x3 sub-matrix
    double det = m[0][0] * (m[1][1]*m[2][2] - m[1][2]*m[2][1])
               - m[0][1] * (m[1][0]*m[2][2] - m[1][2]*m[2][0])
               + m[0][2] * (m[1][0]*m[2][1] - m[1][1]*m[2][0]);
    outFlipWinding = (det < 0.0);
}

// ---------------------------------------------------------------------------

FbxAxisUnitConverter::FbxAxisUnitConverter(const ConvertOptions& options)
    : mOptions(options)
{
}

FbxAxisUnitConverter::~FbxAxisUnitConverter()
{
    if (mScene)   { mScene->Destroy();   mScene   = nullptr; }
    if (mManager) { mManager->Destroy(); mManager = nullptr; }
}

int FbxAxisUnitConverter::Run()
{
    // --- SDK init ---
    mManager = FbxManager::Create();
    if (!mManager) throw std::runtime_error("Failed to create FbxManager.");

    FbxIOSettings* ios = FbxIOSettings::Create(mManager, IOSROOT);
    mManager->SetIOSettings(ios);

    // --- Import ---
    mScene = FbxFileIO::Import(mManager, mOptions.inputPath);

    // --- Pre-normalization (ルートオブジェクトの補正変換を除去) ---
    PreNormalize();

    // --- Build conversion matrix ---
    BuildConversionMatrix();

    // --- Rewrite GlobalSettings ---
    ApplyGlobalSettings();

    // --- Process scene geometry and transforms ---
    ProcessScene();

    // --- Export ---
    FbxFileIO::Export(mManager, mScene, mOptions.outputPath);
    return 0;
}

void FbxAxisUnitConverter::PreNormalize()
{
    bool hasAxis = mOptions.preNormUp.has_value() && mOptions.preNormForward.has_value();
    bool hasUnit = mOptions.preNormUnit.has_value();
    if (!hasAxis && !hasUnit) return;

    std::cout << "[Info] Pre-normalization: removing baked correction transforms from root children.\n";

    FbxGlobalSettings& gs = mScene->GetGlobalSettings();

    // FBXメタデータから fileSrc 軸系・単位を取得
    auto [fileSrcUp, fileSrcFwd] = DecodeAxisSystem(gs.GetAxisSystem());
    FbxSystemUnit fileSrcUnit = gs.GetSystemUnit();

    // preNorm → fileSrc の変換行列を構築
    FbxAMatrix corrMatrix;
    corrMatrix.SetIdentity();
    bool dummyFlip = false;

    if (hasAxis)
    {
        AxisVector preNormUp    = *mOptions.preNormUp;
        AxisVector preNormFwd   = *mOptions.preNormForward;
        AxisVector preNormRight = ComputeRight(preNormUp, preNormFwd);
        AxisVector fileSrcRight = ComputeRight(fileSrcUp, fileSrcFwd);

        BuildMatrix(preNormUp, preNormFwd, preNormRight,
                    fileSrcUp, fileSrcFwd, fileSrcRight,
                    corrMatrix, dummyFlip);

        const char* axisNames[] = { "X", "Y", "Z" };
        auto axStr = [&](const AxisVector& v) -> std::string {
            return (v.sign < 0 ? "-" : "+") + std::string(axisNames[v.axis]);
        };
        std::cout << "[Info] PreNorm src: up=" << axStr(preNormUp) << " fwd=" << axStr(preNormFwd)
                  << " right=" << axStr(preNormRight) << "\n";
        std::cout << "[Info] FileSrc:     up=" << axStr(fileSrcUp) << " fwd=" << axStr(fileSrcFwd)
                  << " right=" << axStr(fileSrcRight) << "\n";
    }

    FbxAMatrix corrInv = corrMatrix.Inverse();

    // 単位スケールの逆数を計算
    double scaleInv = 1.0;
    if (hasUnit)
    {
        FbxSystemUnit preNormUnit = *mOptions.preNormUnit;
        double corrScale = fileSrcUnit.GetScaleFactor() / preNormUnit.GetScaleFactor();
        scaleInv = 1.0 / corrScale;
        std::cout << "[Info] PreNorm unit: fileSrc=" << fileSrcUnit.GetScaleFactor()
                  << "cm  preNorm=" << preNormUnit.GetScaleFactor()
                  << "cm  corrScale=" << corrScale << "  scaleInv=" << scaleInv << "\n";
    }

    // ルートの直接子ノードに逆補正を適用
    FbxNode* root = mScene->GetRootNode();
    int childCount = root->GetChildCount();
    for (int i = 0; i < childCount; ++i)
    {
        FbxNode* child = root->GetChild(i);
        TransformProcessor::ApplySingleNode(child, corrInv, corrMatrix, scaleInv);

        FbxDouble3 r = child->LclRotation.Get();
        FbxDouble3 s = child->LclScaling.Get();
        FbxDouble3 t = child->LclTranslation.Get();
        std::cout << "[Debug] After pre-norm, node '" << child->GetName() << "':"
                  << " LclRot=("   << r[0] << "," << r[1] << "," << r[2] << ")"
                  << " LclScale=(" << s[0] << "," << s[1] << "," << s[2] << ")"
                  << " LclTrans=(" << t[0] << "," << t[1] << "," << t[2] << ")\n";
    }

    // BuildConversionMatrix() が preNorm 空間を src として使うよう上書き
    if (hasAxis)
    {
        mOptions.srcUp      = mOptions.preNormUp;
        mOptions.srcForward = mOptions.preNormForward;
    }
    if (hasUnit)
    {
        mOptions.srcUnit = mOptions.preNormUnit;
    }
}

void FbxAxisUnitConverter::BuildConversionMatrix()
{
    FbxGlobalSettings& gs = mScene->GetGlobalSettings();

    // --- Determine source axis system ---
    AxisVector srcUp, srcFwd;
    if (mOptions.srcUp.has_value() && mOptions.srcForward.has_value())
    {
        srcUp  = *mOptions.srcUp;
        srcFwd = *mOptions.srcForward;
        std::cout << "[Info] Source axis overridden by arguments.\n";
    }
    else
    {
        FbxAxisSystem fbxSrc = gs.GetAxisSystem();
        auto [decodedUp, decodedFwd] = DecodeAxisSystem(fbxSrc);
        srcUp  = decodedUp;
        srcFwd = decodedFwd;
    }

    // --- Determine destination axis system ---
    AxisVector dstUp, dstFwd;
    bool hasAxisConv = mOptions.dstUp.has_value() && mOptions.dstForward.has_value();
    if (hasAxisConv)
    {
        dstUp  = *mOptions.dstUp;
        dstFwd = *mOptions.dstForward;
    }
    else
    {
        // No destination axis specified -> keep source axis
        dstUp  = srcUp;
        dstFwd = srcFwd;
    }

    // Compute right vectors
    AxisVector srcRight = ComputeRight(srcUp, srcFwd);
    AxisVector dstRight = ComputeRight(dstUp, dstFwd);

    // Logging
    const char* axisNames[] = { "X", "Y", "Z" };
    auto axStr = [&](const AxisVector& v) -> std::string {
        return (v.sign < 0 ? "-" : "+") + std::string(axisNames[v.axis]);
    };
    std::cout << "[Info] Src: up=" << axStr(srcUp) << " fwd=" << axStr(srcFwd)
              << " right=" << axStr(srcRight) << "\n";
    std::cout << "[Info] Dst: up=" << axStr(dstUp) << " fwd=" << axStr(dstFwd)
              << " right=" << axStr(dstRight) << "\n";

    // --- Build rotation/swizzle matrix ---
    BuildMatrix(srcUp, srcFwd, srcRight, dstUp, dstFwd, dstRight,
                mConvMatrix, mFlipWinding);

    std::cout << "[Info] Winding flip: " << (mFlipWinding ? "yes" : "no") << "\n";

    // --- Compute unit scale factor ---
    FbxSystemUnit srcUnit = mOptions.srcUnit.has_value()
                          ? *mOptions.srcUnit
                          : gs.GetSystemUnit();
    FbxSystemUnit dstUnit = mOptions.dstUnit.has_value()
                          ? *mOptions.dstUnit
                          : srcUnit;

    mScaleFactor = srcUnit.GetScaleFactor() / dstUnit.GetScaleFactor();
    std::cout << "[Info] Unit: src=" << srcUnit.GetScaleFactor()
              << "cm  dst=" << dstUnit.GetScaleFactor()
              << "cm  factor=" << mScaleFactor << "\n";
}

void FbxAxisUnitConverter::ApplyGlobalSettings()
{
    FbxGlobalSettings& gs = mScene->GetGlobalSettings();

    if (mOptions.dstUnit.has_value())
        gs.SetSystemUnit(*mOptions.dstUnit);

    if (mOptions.dstUp.has_value() && mOptions.dstForward.has_value())
    {
        FbxAxisSystem dstSys = MakeAxisSystem(*mOptions.dstUp, *mOptions.dstForward);
        gs.SetAxisSystem(dstSys);
    }
}

static void DumpNodeRotations(FbxNode* node, int depth = 0)
{
    if (!node) return;
    std::string indent(depth * 2, ' ');
    FbxDouble3 r = node->LclRotation.Get();
    std::cout << indent << "[Node] " << node->GetName()
              << "  LclRot=(" << r[0] << ", " << r[1] << ", " << r[2] << ")\n";
    for (int i = 0; i < node->GetChildCount(); ++i)
        DumpNodeRotations(node->GetChild(i), depth + 1);
}

void FbxAxisUnitConverter::ProcessScene()
{
    // --- Debug: dump conversion matrix ---
    std::cout << "[Debug] ConvMatrix rows:\n";
    for (int r = 0; r < 4; ++r)
    {
        FbxVector4 row = mConvMatrix.GetRow(r);
        std::cout << "  row" << r << ": ("
                  << row[0] << ", " << row[1] << ", "
                  << row[2] << ", " << row[3] << ")\n";
    }

    std::cout << "[Debug] Node rotations BEFORE transform:\n";
    DumpNodeRotations(mScene->GetRootNode());

    GeometryProcessor geoProc(mFlipWinding);
    geoProc.ProcessScene(mScene, mConvMatrix, mScaleFactor);

    TransformProcessor::ProcessNode(mScene->GetRootNode(), mConvMatrix, mScaleFactor);

    std::cout << "[Debug] Node rotations AFTER transform:\n";
    DumpNodeRotations(mScene->GetRootNode());

    AnimProcessor::ProcessAnimation(mScene, mConvMatrix, mScaleFactor);
}
