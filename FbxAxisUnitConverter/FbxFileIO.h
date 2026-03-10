#pragma once
#include <fbxsdk.h>
#include <string>
#include <stdexcept>

// ---------------------------------------------------------------------------
// FbxFileIO — FBX ファイルの読み書きを担当するユーティリティクラス
//
// FBX SDK のパス引数は UTF-8 を期待する（ヘッダコメント: pFileName_UTF8）。
// Windows では argv / std::string が CP_ACP (Shift-JIS 等) なので、
// FbxAnsiToUTF8 で内部変換してから SDK に渡す。
// ---------------------------------------------------------------------------
class FbxFileIO
{
public:
    // FBX ファイルを読み込み、シーンを返す。
    // manager: 呼び出し側が所有する FbxManager
    // path:    ファイルパス (Windows では CP_ACP エンコーディング)
    // 失敗時は std::runtime_error をスロー
    static FbxScene* Import(FbxManager* manager, const std::string& path);

    // FBX ファイルへシーンを書き出す。
    // path: ファイルパス (Windows では CP_ACP エンコーディング)
    // 失敗時は std::runtime_error をスロー
    static void Export(FbxManager* manager, FbxScene* scene, const std::string& path);

private:
    // CP_ACP パスを FBX SDK が期待する UTF-8 に変換する
    static std::string PathToUtf8(const std::string& path);
};
