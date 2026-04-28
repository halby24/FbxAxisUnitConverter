#include "FbxAxisUnitConverter.h"
#include "GeometryProcessor.h"
#include "TransformProcessor.h"
#include "AnimProcessor.h"
#include <sstream>
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

static const char* AxisName(int axis)
{
    static const char* kNames[] = { "X", "Y", "Z" };
    return kNames[axis];
}

static std::string AxisVecToStr(const AxisVector& v)
{
    return (v.sign < 0 ? "-" : "+") + std::string(AxisName(v.axis));
}

// ---------------------------------------------------------------------------

FbxAxisUnitConverter::FbxAxisUnitConverter(FbxScene* scene,
                                           const ConversionParams& params,
                                           ILogger* logger)
    : mScene(scene)
    , mParams(params)
    , mLogger(logger)
{
    if (!mScene) throw std::runtime_error("FbxAxisUnitConverter: scene is null.");
}

void FbxAxisUnitConverter::Convert()
{
    PreNormalize();
    BuildConversionMatrix();
    ApplyGlobalSettings();
    ProcessScene();
}

void FbxAxisUnitConverter::PreNormalize()
{
    bool hasAxis = mParams.preNormUp.has_value() && mParams.preNormForward.has_value();
    bool hasUnit = mParams.preNormUnit.has_value();
    if (!hasAxis && !hasUnit) return;

    if (mLogger)
        mLogger->Info("Pre-normalization: removing baked correction transforms from root children.");

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
        AxisVector preNormUp    = *mParams.preNormUp;
        AxisVector preNormFwd   = *mParams.preNormForward;
        AxisVector preNormRight = ComputeRight(preNormUp, preNormFwd);
        AxisVector fileSrcRight = ComputeRight(fileSrcUp, fileSrcFwd);

        BuildMatrix(preNormUp, preNormFwd, preNormRight,
                    fileSrcUp, fileSrcFwd, fileSrcRight,
                    corrMatrix, dummyFlip);

        if (mLogger)
        {
            std::ostringstream o1;
            o1 << "PreNorm src: up=" << AxisVecToStr(preNormUp)
               << " fwd=" << AxisVecToStr(preNormFwd)
               << " right=" << AxisVecToStr(preNormRight);
            mLogger->Info(o1.str());

            std::ostringstream o2;
            o2 << "FileSrc:     up=" << AxisVecToStr(fileSrcUp)
               << " fwd=" << AxisVecToStr(fileSrcFwd)
               << " right=" << AxisVecToStr(fileSrcRight);
            mLogger->Info(o2.str());
        }
    }

    FbxAMatrix corrInv = corrMatrix.Inverse();

    // 単位スケールの逆数を計算
    double scaleInv = 1.0;
    if (hasUnit)
    {
        FbxSystemUnit preNormUnit = *mParams.preNormUnit;
        double corrScale = fileSrcUnit.GetScaleFactor() / preNormUnit.GetScaleFactor();
        scaleInv = 1.0 / corrScale;
        if (mLogger)
        {
            std::ostringstream o;
            o << "PreNorm unit: fileSrc=" << fileSrcUnit.GetScaleFactor()
              << "cm  preNorm=" << preNormUnit.GetScaleFactor()
              << "cm  corrScale=" << corrScale << "  scaleInv=" << scaleInv;
            mLogger->Info(o.str());
        }
    }

    // ルートの直接子ノードに逆補正を適用
    FbxNode* root = mScene->GetRootNode();
    int childCount = root->GetChildCount();
    for (int i = 0; i < childCount; ++i)
    {
        FbxNode* child = root->GetChild(i);
        TransformProcessor::ApplySingleNode(child, corrInv, corrMatrix, scaleInv);

        if (mLogger)
        {
            FbxDouble3 r = child->LclRotation.Get();
            FbxDouble3 s = child->LclScaling.Get();
            FbxDouble3 t = child->LclTranslation.Get();
            std::ostringstream o;
            o << "After pre-norm, node '" << child->GetName() << "':"
              << " LclRot=("   << r[0] << "," << r[1] << "," << r[2] << ")"
              << " LclScale=(" << s[0] << "," << s[1] << "," << s[2] << ")"
              << " LclTrans=(" << t[0] << "," << t[1] << "," << t[2] << ")";
            mLogger->Debug(o.str());
        }
    }

    // BuildConversionMatrix() が preNorm 空間を src として使うよう上書き
    if (hasAxis)
    {
        mParams.srcUp      = mParams.preNormUp;
        mParams.srcForward = mParams.preNormForward;
    }
    if (hasUnit)
    {
        mParams.srcUnit = mParams.preNormUnit;
    }
}

void FbxAxisUnitConverter::BuildConversionMatrix()
{
    FbxGlobalSettings& gs = mScene->GetGlobalSettings();

    // --- Determine source axis system ---
    AxisVector srcUp, srcFwd;
    if (mParams.srcUp.has_value() && mParams.srcForward.has_value())
    {
        srcUp  = *mParams.srcUp;
        srcFwd = *mParams.srcForward;
        if (mLogger) mLogger->Info("Source axis overridden by arguments.");
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
    bool hasAxisConv = mParams.dstUp.has_value() && mParams.dstForward.has_value();
    if (hasAxisConv)
    {
        dstUp  = *mParams.dstUp;
        dstFwd = *mParams.dstForward;
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
    if (mLogger)
    {
        std::ostringstream o1;
        o1 << "Src: up=" << AxisVecToStr(srcUp) << " fwd=" << AxisVecToStr(srcFwd)
           << " right=" << AxisVecToStr(srcRight);
        mLogger->Info(o1.str());

        std::ostringstream o2;
        o2 << "Dst: up=" << AxisVecToStr(dstUp) << " fwd=" << AxisVecToStr(dstFwd)
           << " right=" << AxisVecToStr(dstRight);
        mLogger->Info(o2.str());
    }

    // --- Build rotation/swizzle matrix ---
    BuildMatrix(srcUp, srcFwd, srcRight, dstUp, dstFwd, dstRight,
                mConvMatrix, mFlipWinding);

    if (mLogger)
        mLogger->Info(std::string("Winding flip: ") + (mFlipWinding ? "yes" : "no"));

    // --- Compute unit scale factor ---
    FbxSystemUnit srcUnit = mParams.srcUnit.has_value()
                          ? *mParams.srcUnit
                          : gs.GetSystemUnit();
    FbxSystemUnit dstUnit = mParams.dstUnit.has_value()
                          ? *mParams.dstUnit
                          : srcUnit;

    mScaleFactor = srcUnit.GetScaleFactor() / dstUnit.GetScaleFactor();
    if (mLogger)
    {
        std::ostringstream o;
        o << "Unit: src=" << srcUnit.GetScaleFactor()
          << "cm  dst=" << dstUnit.GetScaleFactor()
          << "cm  factor=" << mScaleFactor;
        mLogger->Info(o.str());
    }
}

void FbxAxisUnitConverter::ApplyGlobalSettings()
{
    FbxGlobalSettings& gs = mScene->GetGlobalSettings();

    if (mParams.dstUnit.has_value())
        gs.SetSystemUnit(*mParams.dstUnit);

    if (mParams.dstUp.has_value() && mParams.dstForward.has_value())
    {
        FbxAxisSystem dstSys = MakeAxisSystem(*mParams.dstUp, *mParams.dstForward);
        gs.SetAxisSystem(dstSys);
    }
}

static void DumpNodeRotations(ILogger* logger, FbxNode* node, int depth = 0)
{
    if (!node || !logger) return;
    std::string indent(depth * 2, ' ');
    FbxDouble3 r = node->LclRotation.Get();
    std::ostringstream o;
    o << indent << "[Node] " << node->GetName()
      << "  LclRot=(" << r[0] << ", " << r[1] << ", " << r[2] << ")";
    logger->Debug(o.str());
    for (int i = 0; i < node->GetChildCount(); ++i)
        DumpNodeRotations(logger, node->GetChild(i), depth + 1);
}

void FbxAxisUnitConverter::ProcessScene()
{
    if (mLogger)
    {
        mLogger->Debug("ConvMatrix rows:");
        for (int r = 0; r < 4; ++r)
        {
            FbxVector4 row = mConvMatrix.GetRow(r);
            std::ostringstream o;
            o << "  row" << r << ": ("
              << row[0] << ", " << row[1] << ", "
              << row[2] << ", " << row[3] << ")";
            mLogger->Debug(o.str());
        }

        mLogger->Debug("Node rotations BEFORE transform:");
        DumpNodeRotations(mLogger, mScene->GetRootNode());
    }

    GeometryProcessor geoProc(mFlipWinding, mLogger);
    geoProc.ProcessScene(mScene, mConvMatrix, mScaleFactor);

    TransformProcessor::ProcessNode(mScene->GetRootNode(), mConvMatrix, mScaleFactor);

    if (mLogger)
    {
        mLogger->Debug("Node rotations AFTER transform:");
        DumpNodeRotations(mLogger, mScene->GetRootNode());
    }

    AnimProcessor::ProcessAnimation(mScene, mConvMatrix, mScaleFactor);
}
