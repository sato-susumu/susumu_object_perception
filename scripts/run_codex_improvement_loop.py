#!/usr/bin/env python3
"""Run repeated non-interactive Codex improvement cycles for this repository.

This is the non-hook supervisor. It starts a new `codex exec` process for each
cycle, waits for it to finish, records logs, then starts the next cycle until a
cycle limit is reached or `.codex/auto_improve.stop` exists.
"""

import argparse
import json
import os
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
STOP_FILE = ROOT / '.codex' / 'auto_improve.stop'


BASE_PROMPT = """改善サイクル {cycle} を実施してください。

必須:
- `docs/tasks/README.md` と直近差分を読み、低成績箇所を1つ選ぶ。
- 方針決定時や詰まり時は必ずネットで一次情報・公式ドキュメント・上流 issue を調べる。
- 調査 → 実装 → 評価 → docs更新 → 検証まで進める。
- 並列化できる作業は `multi_tool_use.parallel` で並列化する。
- `git commit` / `git push` はしない。ユーザー変更は戻さない。
- サイクル完了時に、評価値、採用/未採用、次に見る低成績箇所を docs に残す。

この実行は supervisor から呼ばれています。1サイクルが完了したら final で簡潔に結果を返してください。
"""


def run(cmd, stdout_path, env):
    with stdout_path.open('w') as out:
        proc = subprocess.Popen(
            cmd,
            cwd=str(ROOT),
            stdout=out,
            stderr=subprocess.STDOUT,
            text=True,
            env=env,
        )
        return proc.wait()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--cycles', type=int, default=0,
                        help='number of cycles; 0 means run until stop file')
    parser.add_argument('--sleep-sec', type=float, default=5.0)
    parser.add_argument('--log-dir', default='.codex/auto_improve_runs')
    parser.add_argument('--model', default='')
    parser.add_argument('--sandbox', default='danger-full-access',
                        choices=['read-only', 'workspace-write', 'danger-full-access'])
    parser.add_argument('--approval', default='never',
                        choices=['untrusted', 'on-request', 'never'])
    parser.add_argument('--search', action='store_true',
                        help='pass --search to codex exec')
    parser.add_argument('--prompt-file', default='',
                        help='extra prompt text appended to every cycle')
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()

    log_root = ROOT / args.log_dir
    run_id = datetime.now().strftime('%Y%m%d_%H%M%S')
    run_dir = log_root / run_id
    run_dir.mkdir(parents=True, exist_ok=True)

    extra_prompt = ''
    if args.prompt_file:
        extra_prompt = Path(args.prompt_file).read_text()

    env = os.environ.copy()
    env.setdefault('TURTLEBOT3_MODEL', 'waffle')

    cycle = 0
    while True:
        if STOP_FILE.exists():
            print(f'stop file exists: {STOP_FILE}')
            break
        if args.cycles > 0 and cycle >= args.cycles:
            break
        cycle += 1

        prompt = BASE_PROMPT.format(cycle=cycle)
        if extra_prompt.strip():
            prompt += '\n追加指示:\n' + extra_prompt.strip() + '\n'

        final_path = run_dir / f'cycle_{cycle:03d}_final.md'
        stdout_path = run_dir / f'cycle_{cycle:03d}.log'
        prompt_path = run_dir / f'cycle_{cycle:03d}_prompt.md'
        prompt_path.write_text(prompt)

        cmd = [
            'codex', 'exec',
            '-C', str(ROOT),
            '-s', args.sandbox,
            '-a', args.approval,
            '--output-last-message', str(final_path),
        ]
        if args.search:
            cmd.append('--search')
        if args.model:
            cmd.extend(['-m', args.model])
        cmd.append(prompt)

        meta = {
            'cycle': cycle,
            'started_at': datetime.now().isoformat(timespec='seconds'),
            'cmd': cmd[:-1] + ['<prompt>'],
            'prompt_path': str(prompt_path),
            'stdout_path': str(stdout_path),
            'final_path': str(final_path),
        }
        (run_dir / f'cycle_{cycle:03d}_meta.json').write_text(
            json.dumps(meta, indent=2, ensure_ascii=False) + '\n')

        print(f'cycle {cycle}: {" ".join(cmd[:-1])} <prompt>')
        if args.dry_run:
            continue

        rc = run(cmd, stdout_path, env)
        meta['finished_at'] = datetime.now().isoformat(timespec='seconds')
        meta['returncode'] = rc
        (run_dir / f'cycle_{cycle:03d}_meta.json').write_text(
            json.dumps(meta, indent=2, ensure_ascii=False) + '\n')
        print(f'cycle {cycle}: returncode={rc}, log={stdout_path}')

        if rc != 0:
            print('stopping because codex exec returned non-zero')
            return rc

        time.sleep(args.sleep_sec)

    print(f'run directory: {run_dir}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
