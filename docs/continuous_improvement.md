# 継続改善ループ

## 使い方

通常は、Codex に次のように依頼するだけでよい。

```text
改善サイクルをずっと何度も実行して
```

そのセッション内でだけ hook が有効になり、1サイクル終わって turn が止まりそうになるたびに、
次の改善サイクル prompt が自動で投入される。**次の Codex セッションには勝手に持ち越されない**。
別セッションでは、もう一度「改善サイクルを...」と明示した時だけ有効になる。

初回だけ、Codex CLI 側で hook の trust が必要。対話 CLI では `/hooks` を開き、この repo の
`.codex/hooks.json` を確認して trust する。

止める場合は、Codex に次のように指示する。

```text
改善ループを止めて
```

または、ファイルで止める。

```bash
touch .codex/auto_improve.stop
```

ファイル停止を解除する場合:

```bash
rm -f .codex/auto_improve.stop
```

対話セッションに依存せずバックグラウンドで回す場合は、supervisor を明示的に起動する。

```bash
cd ~/ros2_ws/src/susumu_object_perception
python3 scripts/run_codex_improvement_loop.py --cycles 0 --search
```

`--cycles 0` は `.codex/auto_improve.stop` が作られるまで継続する。安全に1回だけ試す場合:

```bash
python3 scripts/run_codex_improvement_loop.py --cycles 1 --search
```

ログと final は `.codex/auto_improve_runs/<timestamp>/` に保存される。

## 仕組み

このパッケージでは、改善サイクルが1回で止まらないように2つの仕組みを用意する。

### 1. Codex Stop hook

`.codex/hooks.json` が `UserPromptSubmit` と `Stop` hook を登録する。

- `UserPromptSubmit`: 「改善サイクル」「どんどん改善」「ずっと何度も」などを含む依頼で、その Codex セッションだけループを arm する
- `Stop`: turn が止まる時、同じ `session_id` で arm されていれば次サイクルの prompt を返す
- 停止: 「改善ループを止めて」と指示するか、`.codex/auto_improve.stop` を作る

この hook は次セッションへ勝手に持ち越さない。`.codex/auto_improve_state.json` に enabled state が
残っていても、`session_id` が変わった Stop では state を無効化して continuation を返さない。

非対話で hook trust を外部で担保する場合だけ、Codex CLI の
`--dangerously-bypass-hook-trust` を使う。

参考:

- https://developers.openai.com/codex/hooks
- https://developers.openai.com/codex/noninteractive

### 2. 非対話 supervisor

`scripts/run_codex_improvement_loop.py` は `codex exec` を1サイクルずつ起動し、終わったら次を起動する。
hook とは独立した明示起動の仕組みなので、起動していない限り次セッションには影響しない。

## サイクル内の必須事項

- `docs/tasks/README.md` と直近差分を読み、低成績箇所を1つ選ぶ。
- 方針決定時や詰まり時は必ずネットで一次情報・公式ドキュメント・上流 issue を調べる。
- 調査、実装、評価、docs 更新、検証まで行う。
- 並列化できる作業は並列化する。
- `git commit` / `git push` はしない。
- サイクル完了時に、評価値、採用/未採用、次に見る低成績箇所を docs に残す。
