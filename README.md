# FbxAxisUnitConverter

FBX ファイルの **軸系**（Up/Forward ベクトル）と**単位**（cm, m など）を変換する C++ CLI ツールです。
変換は座標値を直接書き換える（Swizzle）方式で行うため、追加ノードや変換ノードを挿入せず、DCC ツール上の見た目と Transform 値の整合性を保ちます。

## 機能

- **軸変換** — 任意の Up/Forward ベクトルの組み合わせに変換（例: Y-up/-Z-forward → Z-up/Y-forward）
- **単位変換** — FBX SDK 定義の単位（mm / cm / dm / m / km / inch / foot / yard / mile）へのスケール変換
- **ジオメトリ焼き込み** — 頂点、法線、Tangent、Binormal の座標変換
- **ノード Transform 変換** — LclTranslation / LclRotation / LclScaling の書き換え
- **共有メッシュ対応** — インスタンス化されたメッシュの重複変換を防止
- **入力軸/単位の自動検出** — FBX の GlobalSettings から読み取り（引数で上書き可能）
- **Pre-normalization** — Blender 等の DCC ツールがルートオブジェクトに焼き込んだ補正変換を除去してから通常変換を行う

## ビルド要件

| 項目 | バージョン |
|------|-----------|
| Autodesk FBX SDK | 2020.x 以降（スタティックリンク） |
| Visual Studio | 2022 (Build Tools v143) |
| C++ 規格 | C++17 |
| プラットフォーム | x64 |

FBX SDK のルートは CMake 変数または環境変数 `FBXSDK_ROOT` で指定する。
未指定の場合は Windows 標準インストール先（`C:\Program Files\Autodesk\FBX\FBX SDK\2020.3.9`）を使う。

### macOS での追加要件

| 項目 | バージョン |
|------|-----------|
| OS / アーキテクチャ | macOS (arm64 で検証) |
| Autodesk FBX SDK | 2020.x 以降（`/Applications/Autodesk/FBX SDK/<バージョン>/` にインストール） |
| CMake | 3.20 以降 |
| コンパイラ | AppleClang（Xcode 付属） |
| ビルドバックエンド | Ninja（例） |

- FBX SDK のルートは `cmake/FindFBXSDK.cmake` が自動検出する（`/Applications/Autodesk/FBX SDK/` 配下の最新バージョン）。
  標準外の場所にインストールした場合は CMake 変数または環境変数 `FBXSDK_ROOT` で指定する。
- macOS では OpenMP を有効化しない（逐次実行）。FBX SDK はスタティックリンクのため、実行時に追加の dylib は不要。

## ビルド

### Windows でのビルド

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
```

生成器の指定は親リポジトリの `build_addon.py` と同じである。
出力: `build\FbxAxisUnitConverter\Release\FbxAxisUnitConverter.exe`
（Visual Studio 生成器は構成名サブディレクトリを追加する。親リポジトリ経由でビルドする場合は `CMAKE_RUNTIME_OUTPUT_DIRECTORY` により `<ビルドディレクトリ>/bin/Release` 配下に集約される）

FBX SDK はスタティックリンクのため、実行時に追加の DLL は不要です。

なお、上記 Windows 手順は macOS 上で検証していない。生成器の指定と出力配置の規則は親リポジトリのビルドスクリプトおよび CMake の既定動作に基づく。

### macOS でのビルド

```bash
cmake -S . -B build-release -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build-release
ctest --test-dir build-release --output-on-failure
```

Debug 構成も同様に `-DCMAKE_BUILD_TYPE=Debug` で別のビルドディレクトリに作成する。
FBX SDK は自動検出される。標準外の場所にある場合は `-DFBXSDK_ROOT="/path/to/FBX SDK/x.y.z"` を `cmake` に渡すか、環境変数 `FBXSDK_ROOT` を設定する。

## 使い方

```
FbxAxisUnitConverter -i <input.fbx> -o <output.fbx> [options]

Options:
  -i, --input   <path>   入力 FBX ファイルパス（必須）
  -o, --output  <path>   出力 FBX ファイルパス（必須）
  --up          <axis>   出力の Up ベクトル         例: Y, Z, -Z
  --forward     <axis>   出力の Forward ベクトル    例: -Z, Y
  --unit        <unit>   出力の単位  mm|cm|dm|m|km|inch|foot|yard|mile
  --src-up      <axis>   入力の Up ベクトルを上書き
  --src-forward <axis>   入力の Forward ベクトルを上書き
  --src-unit    <unit>   入力の単位を上書き

Pre-normalization（--pre-norm-* と --src-* は排他）:
  --pre-norm-up      <axis>   補正前の実効 Up 軸    例: Z
  --pre-norm-forward <axis>   補正前の実効 Forward 軸  例: Y
  --pre-norm-unit    <unit>   補正前の実効単位       例: m
```

`--up` と `--forward` は **両方同時に指定するか、両方省略**してください。
Up ベクトルと Forward ベクトルは**直交**していなければなりません。
`--pre-norm-up` と `--pre-norm-forward` も同様のルールが適用されます。

### 使用例

```bash
# Y-up / -Z-forward（OpenGL/Unity）へ変換、単位を m に
FbxAxisUnitConverter -i model.fbx -o out.fbx --up Y --forward -Z --unit m

# Z-up / Y-forward（3ds Max / Maya Z-up 系）へ変換
FbxAxisUnitConverter -i model.fbx -o out.fbx --up Z --forward Y

# 単位だけ cm → m に変換（軸変換なし）
FbxAxisUnitConverter -i model.fbx -o out.fbx --unit m

# 入力 FBX の GlobalSettings が壊れている場合に軸を強制指定
FbxAxisUnitConverter -i model.fbx -o out.fbx \
    --src-up Y --src-forward -Z \
    --up Z --forward Y

# Blender デフォルト出力（メタデータ: Y-up/cm、実態: Z-up/m）を Y-up/cm に変換
FbxAxisUnitConverter -i blender_export.fbx -o out.fbx \
    --pre-norm-up Z --pre-norm-forward Y --pre-norm-unit m \
    --up Y --forward -Z --unit cm
```

### Forward ベクトルの定義

`--forward` は「キャラクターやオブジェクトが向く方向（画面奥方向）」を指定します。

| エンジン/ツール | --up | --forward |
|----------------|------|-----------|
| Unity / OpenGL | Y    | -Z        |
| Blender        | Z    | Y         |
| 3ds Max        | Z    | Y         |
| Maya (Y-up)    | Y    | -Z        |
| Maya (Z-up)    | Z    | -Y        |

### Pre-normalization について

Blender などの DCC ツールは、内部座標系（例: Z-up, m）と異なる設定で FBX をエクスポートする場合、FBX メタデータの軸/単位を書き換えたうえで、シーン内の各ルートオブジェクトに補正変換（LclRotation, LclScaling など）を焼き込みます。

`--pre-norm-*` オプションを使うと、変換パイプラインの先頭でその補正変換を自動的に除去し、その後の通常変換が正しい実効軸/単位を起点として動作するようになります。

**Blender の FBX エクスポート（デフォルト設定）の場合:**
- メタデータ上の軸系: Y-up / cm
- 実際のモデルは Z-up / m で作られており、各ルートオブジェクトに `LclRotation=(-90, 0, 0)`, `LclScaling=(100, 100, 100)` が焼き込まれている

```bash
FbxAxisUnitConverter -i blender_export.fbx -o out.fbx \
    --pre-norm-up Z --pre-norm-forward Y --pre-norm-unit m \
    --up Y --forward -Z --unit cm
```

## 制限事項

- **左手座標系**（DirectX 等）への変換は非対応（右手座標系のみ）
- **アニメーションカーブ**の変換は未実装（TODO）
- **スキンメッシュ**（FbxPose / FbxCluster）の変換は未実装（現時点では非対応）
- Skew（斜行）を含む Transform は正しく変換できない場合があります

## プロジェクト構成

```
FbxAxisUnitConverter/
├── CMakeLists.txt           # トップレベル定義（C++17、FBX SDK 検出、テスト登録）
├── cmake/
│   └── FindFBXSDK.cmake     # FBX SDK 検出モジュール
├── FbxAxisUnitConverter/    # 本体（ライブラリ + CLI 実行ファイル）
│   ├── CMakeLists.txt
│   ├── main.cpp               # エントリポイント
│   ├── ConvertOptions.h       # データ構造（AxisVector, ConvertOptions）
│   ├── ArgumentParser.h/cpp   # CLI 引数パース・バリデーション
│   ├── FbxAxisUnitConverter.h/cpp  # メインコントローラ
│   ├── FbxFileIO.h/cpp        # FBX 入出力
│   ├── GeometryProcessor.h/cpp     # メッシュ頂点・法線変換
│   ├── TransformProcessor.h/cpp    # ノード Transform 変換
│   ├── Logger.h               # ロガー注入用インターフェース
│   └── AnimProcessor.h/cpp         # アニメーション変換（TODO）
├── FbxAxisUnitConverterTests/  # GoogleTest 群
│   ├── CMakeLists.txt
│   ├── TestMain.cpp
│   ├── TestHelpers.h
│   └── Test_*.cpp             # 各種変換・日本語パスのテスト
└── third_party/
    └── googletest             # サブモジュール
```

## ライセンス

[MIT License](LICENSE)
