---
name: development-rules
description: Apply strict minimal-change development rules for coding tasks, especially Streamlit debugging, PoC work, notebooks, small apps, and repository edits. Use when the user asks Codex to follow development rules, keep changes minimal, avoid unsolicited UI or analysis additions, fix Streamlit startup issues, or work on PoC-stage code where validation matters more than polish.
---

# Development Rules

## Core Rule

Limit changes to the requested scope. Prefer the smallest edit that satisfies the user's request.

## Minimal Change Principle

- Modify only what the user requested.
- Do not add new features unless explicitly requested.
- Do not improve UI unless explicitly requested.
- Do not add analysis functionality unless explicitly requested.
- Do not create demo data unless explicitly requested.
- Do not turn a PoC into a polished deliverable unless explicitly requested.

## Streamlit Tasks

When asked to check or fix Streamlit startup, prioritize in this order:

1. Resolve errors.
2. Confirm the app starts.
3. Perform the smallest display check needed.

Add analysis screens, charts, dashboards, controls, or layout improvements only when the user explicitly asks for them.

## PoC Tasks

For PoC-stage work, prioritize validation over completeness.

- Implement only what is necessary to test the current hypothesis.
- Do not implement beyond MVP scope.
- Do not prebuild features that may be useful later.

## PAI最終課題 特例（2026年11月2日提出まで）

この特例は、PAI最終課題のRoomba 680・YOLO・ROS2・Pico W開発にのみ適用する。通常のMinimal Change Principleは維持し、次の範囲に限って例外を認める。

### 1. 実機統合に必要な実装

- 既存機能の統合、試験自動化、診断ログ、撮影補助、提出物作成に必要な変更を許可する。
- 変更の必要性と影響範囲を説明する。

### 2. PoCの制限を一部解除

- MVP以上の実装を一律に禁止しない。
- AI認識から実機ロボット動作までの統合を、今回の明示的な開発目標とする。
- オドメトリ融合、BRC自動起床など、提出に不要な拡張を無断で追加しない。

### 3. 成果物の作成

提出用動画、PDFレポート、GitHub公開資料、実験ログ、再現手順書の作成を許可する。

### 4. GO/NO-GO判定

- 開発作業の継続と、実機走行の許可を分離する。
- NO-GOの場合は、具体的な未達条件、根拠、解除に必要な最小作業を提示する。
- 過去のPASS結果を尊重する。ただし、安全に影響する構成変更があれば必要な再確認を実施する。

### 5. 維持する安全条件

次の条件は緩和しない。

- ARMゲート
- STOPラッチ
- Picoの500ms watchdog
- UART異常時の停止・走行禁止
- 実機20mm/s速度上限
- 短縮E試験の最大1秒制限
- 50cm近傍ガード
- 姿勢鮮度判定
- 人間による実機試験の最終承認

### 6. 有効期間

- 2026年11月2日の課題提出まで有効とする。
- 提出後は通常のDevelopment Rulesに戻す。
- この特例は恒久化しない。

### GitHub記録

- 元のMarkdownを保存する。
- 変更はこの特例節の追加を原則とする。
- 差分を確認し、必要な回帰テストを実施する。
- GitHubの専用ブランチにコミットし、PRを作成して変更内容を報告する。
- ローカル専用ルールと、GitHubに記録した内容および実際の適用状態を区別する。

既存の安全機構や走行ロックは変更しない。
