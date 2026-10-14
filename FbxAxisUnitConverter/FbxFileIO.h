#pragma once
#include "Logger.h"
#include <fbxsdk.h>
#include <string>
#include <stdexcept>

// ---------------------------------------------------------------------------
// FbxFileIO — FBX ファイルの読み書きを担当するユーティリティクラス
//
// FBX SDK のパス引数は UTF-8 を期待する（ヘッダコメント: pFileName_UTF8）。
// Windows では argv / std::string が CP_ACP (Shift-JIS 等) なので、
// FbxAnsiToUTF8 で内部変換してから SDK に渡す。
//
// logger は非所有, nullptr 可（その場合ログ出力なし）。
// ---------------------------------------------------------------------------
class FbxFileIO
{
public:
    // FBX ファイルを読み込み、シーンを返す。
    // SDK の読み込み成功に加え、Import の成否を確認する。
    // 失敗時は std::runtime_error をスローし、"Loaded" は出力しない。
    static FbxScene* Import(FbxManager* manager,
                            const std::string& path,
                            ILogger* logger = nullptr);

    // FBX ファイルへ通常ファイルとしてシーンを書き出す。
    // SDK の成功に加え、出力後に通常ファイルとして存在することを確認し、
    // 満たさなければ std::runtime_error をスローする。
    // すべての検証を通った場合のみ "Saved" を出力する。
    // 失敗時は std::runtime_error をスロー
    static void Export(FbxManager* manager,
                       FbxScene* scene,
                       const std::string& path,
                       ILogger* logger = nullptr);

private:
    // CP_ACP パスを FBX SDK が期待する UTF-8 に変換する
    static std::string PathToUtf8(const std::string& path);
};
