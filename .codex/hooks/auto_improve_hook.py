#!/usr/bin/env python3
"""Codex lifecycle hook for this repo's continuous improvement loop.

UserPromptSubmit arms/disarms the loop from natural-language prompts.
Stop keeps the same Codex session moving by returning a continuation prompt.
Runtime state is intentionally kept out of git via .gitignore.
"""

import json
import re
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
CONFIG_PATH = ROOT / '.codex' / 'auto_improve_config.json'
STATE_PATH = ROOT / '.codex' / 'auto_improve_state.json'


DEFAULT_CONFIG = {
    'enabled_by_default': False,
    'scope': 'session',
    'max_cycles': 0,
    'stop_file': '.codex/auto_improve.stop',
    'trigger_patterns': ['改善サイクル', 'どんどん改善', 'continuous improvement'],
    'stop_patterns': ['改善ループを止めて', 'auto improve off'],
}


def load_json(path, default):
    try:
        return json.loads(path.read_text())
    except FileNotFoundError:
        return dict(default)
    except Exception:
        return dict(default)


def write_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False) + '\n')


def hook_input():
    raw = sys.stdin.read()
    if not raw.strip():
        return {}
    return json.loads(raw)


def matches_any(text, patterns):
    return any(re.search(pat, text, re.IGNORECASE) for pat in patterns)


def output(obj):
    sys.stdout.write(json.dumps(obj, ensure_ascii=False) + '\n')


def state_with_defaults(config):
    state = load_json(STATE_PATH, {})
    if 'enabled' not in state:
        state['enabled'] = bool(config.get('enabled_by_default', False))
    state.setdefault('cycle', 0)
    state.setdefault('objective', '')
    state.setdefault('armed_at', None)
    state.setdefault('session_id', None)
    return state


def arm_or_disarm(event):
    config = load_json(CONFIG_PATH, DEFAULT_CONFIG)
    prompt = str(event.get('prompt') or '')
    state = state_with_defaults(config)

    if matches_any(prompt, config.get('stop_patterns', [])):
        state['enabled'] = False
        state['session_id'] = None
        state['disabled_at'] = time.time()
        state['disabled_by_prompt'] = prompt[:500]
        write_json(STATE_PATH, state)
        output({
            'hookSpecificOutput': {
                'hookEventName': 'UserPromptSubmit',
                'additionalContext': (
                    'Continuous improvement loop is disabled for this repo. '
                    'Remove .codex/auto_improve.stop if it exists before re-enabling.'
                )
            }
        })
        return

    if matches_any(prompt, config.get('trigger_patterns', [])):
        state['enabled'] = True
        state['objective'] = prompt.strip()
        state['armed_at'] = time.time()
        state['session_id'] = event.get('session_id')
        state['cycle'] = 0
        write_json(STATE_PATH, state)
        output({
            'hookSpecificOutput': {
                'hookEventName': 'UserPromptSubmit',
                'additionalContext': (
                    'Continuous improvement loop is armed for this Codex session only. '
                    'After each Stop in this session, the repo hook will request the next cycle. '
                    'To stop it, say "改善ループを止めて" or create .codex/auto_improve.stop.'
                )
            }
        })
        return

    # Valid JSON with no changes. Plain text is invalid for Stop, but OK here;
    # returning JSON keeps behavior uniform.
    output({})


def next_cycle_prompt(state):
    cycle = int(state.get('cycle', 0)) + 1
    objective = state.get('objective') or '各種タスクの低成績箇所を改善する'
    return f"""改善サイクル {cycle} を開始してください。

目的:
{objective}

必須手順:
1. `docs/tasks/README.md` と直近の変更を確認し、低成績箇所を1つ選ぶ。
2. 方針決定時、またはローカル情報だけで解けない時は必ずネットで一次情報・公式ドキュメント・上流 issue を調べ、参照先を docs または最終報告に残す。
3. 調査 → 実装 → 評価 → docs 更新 → 静的/ライブ検証まで行う。
4. 並列でできる file read/search/build/評価は `multi_tool_use.parallel` で並列化する。
5. `git commit` / `git push` はしない。ユーザー変更は戻さない。
6. サイクル完了時は、採用/未採用、評価値、次の低成績箇所を docs に残す。

この hook が有効な限り、完了後は停止せず次の改善サイクルへ進みます。
停止するには「改善ループを止めて」と指示するか、`.codex/auto_improve.stop` を作成してください。
"""


def continue_on_stop(event):
    config = load_json(CONFIG_PATH, DEFAULT_CONFIG)
    state = state_with_defaults(config)
    stop_path = ROOT / str(config.get('stop_file', '.codex/auto_improve.stop'))

    if stop_path.exists():
        state['enabled'] = False
        state['session_id'] = None
        state['disabled_at'] = time.time()
        state['disabled_reason'] = f'stop file exists: {stop_path}'
        write_json(STATE_PATH, state)
        output({'systemMessage': 'Continuous improvement loop stopped by stop file.'})
        return

    if not state.get('enabled', False):
        output({})
        return

    if config.get('scope', 'session') == 'session':
        armed_session = state.get('session_id')
        current_session = event.get('session_id')
        if not armed_session or armed_session != current_session:
            state['enabled'] = False
            state['session_id'] = None
            state['disabled_at'] = time.time()
            state['disabled_reason'] = 'session changed or session was not recorded'
            state['disabled_previous_session_id'] = armed_session
            state['disabled_current_session_id'] = current_session
            write_json(STATE_PATH, state)
            output({})
            return

    max_cycles = int(config.get('max_cycles') or 0)
    if max_cycles > 0 and int(state.get('cycle', 0)) >= max_cycles:
        state['enabled'] = False
        state['session_id'] = None
        state['disabled_at'] = time.time()
        state['disabled_reason'] = f'max_cycles reached: {max_cycles}'
        write_json(STATE_PATH, state)
        output({'systemMessage': f'Continuous improvement loop stopped at max_cycles={max_cycles}.'})
        return

    prompt = next_cycle_prompt(state)
    state['cycle'] = int(state.get('cycle', 0)) + 1
    state['last_continued_at'] = time.time()
    state['last_session_id'] = event.get('session_id')
    state['last_turn_id'] = event.get('turn_id')
    write_json(STATE_PATH, state)
    output({
        'decision': 'block',
        'reason': prompt,
    })


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else ''
    event = hook_input()
    if mode == 'arm':
        arm_or_disarm(event)
    elif mode == 'stop':
        continue_on_stop(event)
    else:
        output({})


if __name__ == '__main__':
    main()
