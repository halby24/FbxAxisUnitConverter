#include "FbxFileIO.h"

#include <filesystem>

FbxScene* FbxFileIO::Import(FbxManager* manager, const std::string& path, ILogger* logger)
{
    std::string utf8Path = PathToUtf8(path);

    FbxScene* scene = FbxScene::Create(manager, "Scene");
    FbxImporter* importer = FbxImporter::Create(manager, "");
    if (!importer->Initialize(utf8Path.c_str(), -1, manager->GetIOSettings()))
    {
        std::string err = importer->GetStatus().GetErrorString();
        importer->Destroy();
        scene->Destroy();
        throw std::runtime_error(
            std::string("Failed to open input file: ") + path + "\n" + err);
    }
    bool importOk = importer->Import(scene);
    // Destroy 後はステータスが無効になるため、先にコピーする。
    std::string sdkStatus = importer->GetStatus().GetErrorString();
    importer->Destroy();

    if (!importOk)
    {
        scene->Destroy();
        std::string message = std::string("Failed to read input file: ") + path;
        if (!sdkStatus.empty())
            message += "\n" + sdkStatus;
        throw std::runtime_error(message);
    }

    if (logger) logger->Info("Loaded: " + path);
    return scene;
}

void FbxFileIO::Export(FbxManager* manager, FbxScene* scene, const std::string& path, ILogger* logger)
{
    std::string utf8Path = PathToUtf8(path);

    FbxExporter* exporter = FbxExporter::Create(manager, "");
    if (!exporter->Initialize(utf8Path.c_str(), -1, manager->GetIOSettings()))
    {
        // Destroy 後はステータスが無効になるため、先にコピーする。
        std::string sdkStatus = exporter->GetStatus().GetErrorString();
        exporter->Destroy();
        std::string message = std::string("Failed to open output file: ") + path;
        if (!sdkStatus.empty())
            message += "\n" + sdkStatus;
        throw std::runtime_error(message);
    }
    bool exportOk = exporter->Export(scene);
    // Destroy 後はステータスが無効になるため、先にコピーする。
    std::string sdkStatus = exporter->GetStatus().GetErrorString();
    exporter->Destroy();

    // ADR: Export の戻り値だけでは既存ディレクトリへの偽成功
    // （Export が true を返すのに何も書かない）を検出できないため、
    // Destroy 後に出力ファイルの存在を postcondition として検証する。
    // Windows で UTF-8 パスをそのまま path に渡さないよう u8path を使う。
    // error_code 版で例外を出さず、error_code と戻り値の両方を見る。
    std::error_code ec;
    bool isRegularFile =
        std::filesystem::is_regular_file(std::filesystem::u8path(utf8Path), ec);
    bool fileOk = !ec && isRegularFile;

    if (!exportOk || !fileOk)
    {
        std::string message = std::string("Failed to write output file: ") + path;
        if (!sdkStatus.empty())
            message += "\n" + sdkStatus;
        if (!fileOk)
            message += "\nOutput is not a regular file after export.";
        throw std::runtime_error(message);
    }

    if (logger) logger->Info("Saved: " + path);
}

std::string FbxFileIO::PathToUtf8(const std::string& path)
{
#ifdef _WIN32
    char* utf8 = nullptr;
    FbxAnsiToUTF8(path.c_str(), utf8);
    if (!utf8) return path;
    std::string result(utf8);
    FbxFree(utf8);
    return result;
#else
    return path;
#endif
}
