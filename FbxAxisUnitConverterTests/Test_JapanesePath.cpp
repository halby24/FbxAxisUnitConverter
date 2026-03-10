// Test_JapanesePath.cpp
//
// FbxFileIO の日本語パス対応テスト。
// FbxFileIO::Export → FbxFileIO::Import の往復で日本語ディレクトリ名・
// 日本語ファイル名が正しく扱えることを確認する。
//
// Windows の CP_ACP が日本語を表現できない環境（英語 Windows + CP1252 等）では
// GTEST_SKIP() で自動スキップされる。

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

#include "TestHelpers.h"
#include "FbxFileIO.h"
#include <gtest/gtest.h>
#include <string>

#ifdef _WIN32

// ---------------------------------------------------------------------------
// Wide (UTF-16) → CP_ACP narrow string 変換
// CP_ACP が日本語を表現できない場合は空文字列を返す
// ---------------------------------------------------------------------------
static std::string WideToAcp(const std::wstring& wide)
{
    if (wide.empty()) return {};

    UINT acp = GetACP();
    if (acp == CP_UTF8)
    {
        int needed = WideCharToMultiByte(
            CP_UTF8, 0, wide.c_str(), -1, nullptr, 0, nullptr, nullptr);
        if (needed <= 0) return {};
        std::string s(needed - 1, '\0');
        WideCharToMultiByte(
            CP_UTF8, 0, wide.c_str(), -1, s.data(), needed, nullptr, nullptr);
        return s;
    }

    BOOL usedDefault = FALSE;
    int needed = WideCharToMultiByte(
        CP_ACP, WC_NO_BEST_FIT_CHARS,
        wide.c_str(), -1, nullptr, 0, nullptr, &usedDefault);
    if (needed <= 0 || usedDefault) return {};

    std::string s(needed - 1, '\0');
    WideCharToMultiByte(
        CP_ACP, WC_NO_BEST_FIT_CHARS,
        wide.c_str(), -1, s.data(), needed, nullptr, &usedDefault);
    if (usedDefault) return {};
    return s;
}

// ---------------------------------------------------------------------------
// テスト用のインメモリ FBX シーンを作成する（ノード1個）
// ---------------------------------------------------------------------------
static FbxScene* CreateMinimalScene(FbxManager* mgr)
{
    FbxScene* scene = FbxScene::Create(mgr, "MinScene");
    FbxNode*  node  = FbxNode::Create(scene, "Node1");
    node->LclTranslation.Set(FbxDouble3(1, 2, 3));
    scene->GetRootNode()->AddChild(node);
    return scene;
}

// ---------------------------------------------------------------------------
// テスト fixture: FbxManager の RAII 管理 + CP_ACP スキップ判定
// ---------------------------------------------------------------------------
class JapanesePathTest : public ::testing::Test
{
protected:
    FbxManager* mgr = nullptr;

    void SetUp() override
    {
        mgr = FbxManager::Create();
        FbxIOSettings* ios = FbxIOSettings::Create(mgr, IOSROOT);
        mgr->SetIOSettings(ios);
    }

    void TearDown() override
    {
        if (mgr) { mgr->Destroy(); mgr = nullptr; }
    }

    // CP_ACP で日本語表現可能かチェック。不可なら空文字列を返す
    static std::string TryAcpPath(const std::wstring& wide)
    {
        return WideToAcp(wide);
    }
};

// ---------------------------------------------------------------------------
// テスト: 日本語ディレクトリ内で Export → Import が成功する
// ---------------------------------------------------------------------------
TEST_F(JapanesePathTest, ExportImportWithJapaneseDirectory)
{
    // 一時ディレクトリにサブフォルダを作成
    wchar_t tempW[MAX_PATH];
    GetTempPathW(MAX_PATH, tempW);
    std::wstring dirW = std::wstring(tempW) + L"fbx_jptest_テスト\\";
    CreateDirectoryW(dirW.c_str(), nullptr);

    std::wstring fileW = dirW + L"入力.fbx";
    std::string  fileA = TryAcpPath(fileW);
    if (fileA.empty())
        GTEST_SKIP() << "CP" << GetACP() << " cannot represent Japanese characters.";

    // Export (FbxFileIO 経由)
    FbxScene* scene = CreateMinimalScene(mgr);
    ASSERT_NO_THROW(FbxFileIO::Export(mgr, scene, fileA))
        << "FbxFileIO::Export failed for Japanese directory path";

    // Import (FbxFileIO 経由)
    scene->Destroy();
    FbxScene* loaded = nullptr;
    ASSERT_NO_THROW(loaded = FbxFileIO::Import(mgr, fileA))
        << "FbxFileIO::Import failed for Japanese directory path";
    ASSERT_NE(loaded, nullptr);

    // ノードが残っていることを確認
    EXPECT_EQ(loaded->GetRootNode()->GetChildCount(), 1);
    FbxDouble3 t = loaded->GetRootNode()->GetChild(0)->LclTranslation.Get();
    ExpectDouble3Near(t, FbxDouble3(1, 2, 3));

    loaded->Destroy();

    // クリーンアップ
    DeleteFileW(fileW.c_str());
    RemoveDirectoryW(dirW.c_str());
}

// ---------------------------------------------------------------------------
// テスト: ASCII ディレクトリ内の日本語ファイル名で Export → Import が成功する
// ---------------------------------------------------------------------------
TEST_F(JapanesePathTest, ExportImportWithJapaneseFilename)
{
    wchar_t tempW[MAX_PATH];
    GetTempPathW(MAX_PATH, tempW);
    std::wstring fileW = std::wstring(tempW) + L"fbx_jptest_出力ファイル.fbx";
    std::string  fileA = TryAcpPath(fileW);
    if (fileA.empty())
        GTEST_SKIP() << "CP" << GetACP() << " cannot represent Japanese characters.";

    // Export
    FbxScene* scene = CreateMinimalScene(mgr);
    ASSERT_NO_THROW(FbxFileIO::Export(mgr, scene, fileA));

    // Import
    scene->Destroy();
    FbxScene* loaded = nullptr;
    ASSERT_NO_THROW(loaded = FbxFileIO::Import(mgr, fileA));
    ASSERT_NE(loaded, nullptr);

    EXPECT_EQ(loaded->GetRootNode()->GetChildCount(), 1);
    FbxDouble3 t = loaded->GetRootNode()->GetChild(0)->LclTranslation.Get();
    ExpectDouble3Near(t, FbxDouble3(1, 2, 3));

    loaded->Destroy();
    DeleteFileW(fileW.c_str());
}

#else   // !_WIN32

TEST(JapanesePath, SkippedOnNonWindows)
{
    GTEST_SKIP() << "Japanese path tests are Windows-specific.";
}

#endif  // _WIN32
