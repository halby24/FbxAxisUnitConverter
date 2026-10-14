// Test_FileIO.cpp
//
// FbxFileIO の書き出し失敗時の偽成功を防ぐための結合テスト。
// 実SDKを使い、Export の戻り値無視・出力ファイル未検証の回帰を検出する。
// Import の失敗時に例外となり "Loaded" を出さないことも保証する。

#include "TestHelpers.h"
#include "FbxFileIO.h"
#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <random>
#include <string>
#include <vector>

namespace {

// 一時ディレクトリを一意に作成し、成功・失敗どちらの場合も後始末する。
// デストラクタで remove_all するため、テストが失敗しても残骸を残さない。
class ScopedTempDir
{
public:
    explicit ScopedTempDir(const std::string& namePrefix)
    {
        static std::atomic<unsigned> serial{0};
        std::random_device random;
        auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
        dir = std::filesystem::temp_directory_path()
            / (namePrefix + std::to_string(stamp)
                + "_" + std::to_string(random())
                + "_" + std::to_string(serial.fetch_add(1, std::memory_order_relaxed)));
        std::error_code ec;
        if (!std::filesystem::create_directory(dir, ec) || ec)
            throw std::runtime_error("一時ディレクトリを作成できない: " + dir.string());
    }

    ~ScopedTempDir()
    {
        std::error_code ec;
        std::filesystem::remove_all(dir, ec);
    }

    ScopedTempDir(const ScopedTempDir&) = delete;
    ScopedTempDir& operator=(const ScopedTempDir&) = delete;

    std::filesystem::path dir;
};

// メッセージを記録する ILogger 実装。
// "Saved" / "Loaded" の有無で偽成功（失敗したのに成功ログ）を検出する。
class RecordingLogger : public ILogger
{
public:
    void Info(const std::string& msg) override { infos.push_back(msg); }
    void Debug(const std::string&) override {}
    void Warn(const std::string&) override {}
    void Error(const std::string&) override {}

    bool HasInfoContaining(const std::string& text) const
    {
        for (const auto& m : infos)
            if (m.find(text) != std::string::npos)
                return true;
        return false;
    }

    std::vector<std::string> infos;
};

// テスト用のインメモリ FBX シーンを作成する（ノード1個）。
// Create したシーンは利用後に呼び出し側で Destroy() する。
FbxScene* CreateMinimalScene(FbxManager* mgr)
{
    FbxScene* scene = FbxScene::Create(mgr, "MinScene");
    FbxNode* node = FbxNode::Create(scene, "Node1");
    node->LclTranslation.Set(FbxDouble3(1, 2, 3));
    scene->GetRootNode()->AddChild(node);
    return scene;
}

}  // namespace

// テスト fixture: FbxManager の RAII 管理
class FileIOTest : public ::testing::Test
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
};

// 親ディレクトリが無いパスへの Export が例外となり、偽の "Saved" を出さないこと。
// 書き出し失敗時の偽成功（exit 0 でファイル無し）の回帰を防ぐ。
TEST_F(FileIOTest, ExportFailsWhenParentDirectoryIsMissing)
{
    ScopedTempDir tmp("fbx_fileio_");
    RecordingLogger logger;
    std::filesystem::path file = tmp.dir / "no_such_dir" / "out.fbx";

    FbxScene* scene = CreateMinimalScene(mgr);
    EXPECT_THROW(FbxFileIO::Export(mgr, scene, file.string(), &logger),
                 std::runtime_error);
    scene->Destroy();

    EXPECT_FALSE(logger.HasInfoContaining("Saved"));
    EXPECT_FALSE(std::filesystem::exists(file));
}

// 出力先が既存ディレクトリのパスへの Export が例外となり、偽の "Saved" を出さないこと。
// SDK が Export=true を返すのに何も書かない偽成功の回帰を防ぐ。
TEST_F(FileIOTest, ExportFailsWhenOutputIsExistingDirectory)
{
    ScopedTempDir tmp("fbx_fileio_");
    RecordingLogger logger;
    std::filesystem::path dirPath = tmp.dir / "existing_dir";
    ASSERT_TRUE(std::filesystem::create_directory(dirPath));

    FbxScene* scene = CreateMinimalScene(mgr);
    EXPECT_THROW(FbxFileIO::Export(mgr, scene, dirPath.string(), &logger),
                 std::runtime_error);
    scene->Destroy();

    EXPECT_FALSE(logger.HasInfoContaining("Saved"));
    EXPECT_FALSE(std::filesystem::is_regular_file(dirPath));
}

// 妥当な新規パスへの Export が成功し、"Saved" を出して通常ファイルを残すこと。
TEST_F(FileIOTest, ExportCreatesRegularFileAtNewPath)
{
    ScopedTempDir tmp("fbx_fileio_");
    RecordingLogger logger;
    std::filesystem::path file = tmp.dir / "out.fbx";

    FbxScene* scene = CreateMinimalScene(mgr);
    EXPECT_NO_THROW(FbxFileIO::Export(mgr, scene, file.string(), &logger));
    scene->Destroy();

    EXPECT_TRUE(logger.HasInfoContaining("Saved"));
    EXPECT_TRUE(std::filesystem::is_regular_file(file));
}

// 既存の通常ファイルへの上書きが成功し、再 Import で内容が保たれること。
TEST_F(FileIOTest, ExportOverwritesExistingFileAndKeepsContent)
{
    ScopedTempDir tmp("fbx_fileio_");
    RecordingLogger logger;
    std::filesystem::path file = tmp.dir / "overwrite.fbx";

    {
        FbxScene* scene = CreateMinimalScene(mgr);
        ASSERT_NO_THROW(FbxFileIO::Export(mgr, scene, file.string(), &logger));
        scene->Destroy();
    }
    ASSERT_TRUE(std::filesystem::is_regular_file(file));

    {
        FbxScene* scene = CreateMinimalScene(mgr);
        EXPECT_NO_THROW(FbxFileIO::Export(mgr, scene, file.string(), &logger));
        scene->Destroy();
    }
    EXPECT_TRUE(logger.HasInfoContaining("Saved"));

    FbxScene* loaded = nullptr;
    ASSERT_NO_THROW(loaded = FbxFileIO::Import(mgr, file.string(), &logger));
    ASSERT_NE(loaded, nullptr);
    EXPECT_EQ(loaded->GetRootNode()->GetChildCount(), 1);
    FbxDouble3 t = loaded->GetRootNode()->GetChild(0)->LclTranslation.Get();
    ExpectDouble3Near(t, FbxDouble3(1, 2, 3));
    loaded->Destroy();
}

// 非FBXテキストの Import が例外となり、"Loaded" を出さないこと。
TEST_F(FileIOTest, ImportFailsForNonFbxTextFile)
{
    ScopedTempDir tmp("fbx_fileio_");
    RecordingLogger logger;
    std::filesystem::path file = tmp.dir / "not_fbx.txt";
    {
        std::ofstream out(file, std::ios::binary);
        out << "this is not an FBX file\n";
    }

    EXPECT_THROW(FbxFileIO::Import(mgr, file.string(), &logger),
                 std::runtime_error);
    EXPECT_FALSE(logger.HasInfoContaining("Loaded"));
}

// バイナリFBXをヘッダのみ残して切り詰めたファイルの Import が例外となり、"Loaded" を出さないこと。
// importer->Import() が false を返す経路（Initialize 成功・Import 失敗）の回帰を防ぐ。
// 旧コードは Import の戻り値を見ず偽成功（"Loaded" あり・exit 0）した。
TEST_F(FileIOTest, ImportFailsForTruncatedBinaryFbx)
{
    ScopedTempDir tmp("fbx_fileio_");
    RecordingLogger exportLogger;
    RecordingLogger importLogger;
    std::filesystem::path file = tmp.dir / "truncated.fbx";

    // まず最小シーンをバイナリFBXとして出力する（既定IOSettingsはバイナリ）。
    {
        FbxScene* scene = CreateMinimalScene(mgr);
        ASSERT_NO_THROW(FbxFileIO::Export(mgr, scene, file.string(), &exportLogger));
        scene->Destroy();
    }
    ASSERT_TRUE(std::filesystem::is_regular_file(file));

    // ADR: 切り詰めサイズ 64 バイトの根拠。
    // バイナリFBXのマジック（"Kaydara FBX Binary  " 27バイト）のみを残し、
    // シーン本体を欠落させるサイズである。実測（FBX SDK 2020.3.11）では
    // 27〜69バイトの範囲で Initialize=true かつ Import=false となり、
    // 狙った分岐（Import()==false で throw）を踏む。64バイトはその範囲内に
    // あり（下側境界から約37バイト、上側境界から約5バイト）、ヘッダ判定の
    // 境界を避けた位置である。
    // SDK バージョン差で Initialize 失敗経路に落ちても FbxFileIO::Import は
    // 例外を投げるため、このテストの「throw する」検証は保たれる。
    constexpr std::streamsize kTruncatedSize = 64;
    {
        std::ifstream in(file, std::ios::binary);
        ASSERT_TRUE(static_cast<bool>(in));
        std::vector<char> data((std::istreambuf_iterator<char>(in)),
                          std::istreambuf_iterator<char>());
        ASSERT_GT(static_cast<std::streamsize>(data.size()), kTruncatedSize);
        std::ofstream out(file, std::ios::binary | std::ios::trunc);
        ASSERT_TRUE(static_cast<bool>(out));
        out.write(data.data(), kTruncatedSize);
    }

    EXPECT_THROW(FbxFileIO::Import(mgr, file.string(), &importLogger),
                 std::runtime_error);
    EXPECT_FALSE(importLogger.HasInfoContaining("Loaded"));
}
