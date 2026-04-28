#pragma once
#include <iostream>
#include <string>

// ---------------------------------------------------------------------------
// ILogger — 変換処理から呼び出される抽象ロガー
//
// ライブラリ利用者が出力先を差し替えられるようにするための注入ポイント。
// nullptr を渡せば NullLogger と同等の挙動になる（呼び出し側で吸収する）。
// ---------------------------------------------------------------------------
class ILogger
{
public:
    virtual ~ILogger() = default;
    virtual void Info (const std::string& msg) = 0;
    virtual void Debug(const std::string& msg) = 0;
    virtual void Warn (const std::string& msg) = 0;
    virtual void Error(const std::string& msg) = 0;
};

// 何もしないロガー。ライブラリ利用時のデフォルトに使う。
class NullLogger : public ILogger
{
public:
    void Info (const std::string&) override {}
    void Debug(const std::string&) override {}
    void Warn (const std::string&) override {}
    void Error(const std::string&) override {}
};

// 標準出力／標準エラー出力に書き出すロガー。CLI で使う。
class ConsoleLogger : public ILogger
{
public:
    void Info (const std::string& msg) override { std::cout << "[Info] "  << msg << "\n"; }
    void Debug(const std::string& msg) override { std::cout << "[Debug] " << msg << "\n"; }
    void Warn (const std::string& msg) override { std::cout << "[Warn] "  << msg << "\n"; }
    void Error(const std::string& msg) override { std::cerr << "[Error] " << msg << "\n"; }
};
