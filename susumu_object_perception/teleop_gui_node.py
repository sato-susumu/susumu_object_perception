#!/usr/bin/env python3
"""シミュレートした TurtleBot3 用の Tkinter Teleop ＋ 自動巡回 GUI。

小さなウィンドウに以下を備える:

  * 4つの矢印ボタン（上/下/左/右）。ボタンを「押している間」だけロボットが動き、
    離すと止まる。テンキー（および矢印キー）でも同様に操縦できる:
        上    / KP_8 : 前進
        下    / KP_2 : 後退
        左    / KP_4 : 左旋回
        右    / KP_6 : 右旋回
  * ON/OFF が一目で分かる大きなトグルボタン。ON の間はロボットが家を自動巡回する
    （Nav2 経由で部屋のウェイポイントを固定ループで巡る。ランダムに飛び回るのではなく
    部屋から部屋へ順に移動する）。OFF で停止。

手動操縦と自動巡回は Nav2 のコントローラと /cmd_vel を共有するため:
  * 矢印を押すと自動巡回が OFF になり Nav2 ゴールをキャンセルしてから、
    直接 Twist を publish する（手動が優先）。
  * 自動巡回は NavigateToPose ゴールを送り、到達/中断したら次のウェイポイントへ進む。

simulation.launch.py（gui:=true）でシミュレーションと一緒に起動される。
"""

import subprocess
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

try:
    import tkinter as tk
except Exception:  # pragma: no cover - ヘッドレス環境向けガード
    tk = None

# 巡回ウェイポイントは object_seeker_node.py と共有する共通モジュールから取る。
from susumu_object_perception.patrol_waypoints import PATROL_WAYPOINTS

LINEAR_SPEED = 0.22   # m/s   （TurtleBot3 waffle の最大 ~0.26）
ANGULAR_SPEED = 0.9   # rad/s
PUBLISH_HZ = 10.0
ROBOT_ENTITY = 'turtlebot3'   # Gazebo モデル名（spawn したエンティティ名と一致必須）
# 1つのウェイポイントでこの時間を超えたら諦めて次へ進む。1箇所のスタックが
# 巡回全体を止めないようにするため。
WAYPOINT_TIMEOUT_S = 25.0


class TeleopGuiNode(Node):
    def __init__(self):
        super().__init__('teleop_gui')
        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self._initpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # セマンティック物体メモリへのクエリ送信（semantic_query_node が購読）。
        # semantic_memory:=True で起動していなければ誰も読まないだけで無害。
        self._query_pub = self.create_publisher(String, '/semantic_query', 10)
        # 行動層 object_seeker への追従/探索コマンド。
        self._seek_pub = self.create_publisher(String, '/object_seek', 10)

        # 現在の手動コマンド (lin, ang)。(0, 0) は停止を意味する。
        self._manual = (0.0, 0.0)
        self._manual_active = False
        self._auto = False
        self._wp_index = 0
        self._nav_goal_handle = None
        self._nav_busy = False
        self._goal_sent_at = None  # 現在のゴールを開始した実時刻

        # 押下中の手動コマンドを一定レートで再 publish する（diff_drive は Twist の
        # 連続送信を要求する。1回だけだとタイムアウトして止まる）。
        self.create_timer(1.0 / PUBLISH_HZ, self._tick)
        # ウォッチドッグ: AUTO が ON の間は常にゴールが飛んでいる状態を保ち、
        # ロボットがどこかへ向かい続けるようにする。ゴールが失われた / Nav2 が
        # まだ起動していない / 何も処理中でない場合は現在のウェイポイントを（再）送信する。
        self.create_timer(1.0, self._auto_watchdog)

    def send_query(self, text):
        """GUI のクエリ入力を semantic_query_node へ送る（例「椅子」「人」）。"""
        text = (text or '').strip()
        if not text:
            return
        self._query_pub.publish(String(data=text))
        self.get_logger().info(f'semantic query 送信: {text}')

    def send_seek(self, text):
        """object_seeker へ追従/探索コマンドを送る（例「人を追って」「椅子を探して」）。"""
        text = (text or '').strip()
        if not text:
            return
        self._seek_pub.publish(String(data=text))
        self.get_logger().info(f'object_seek 送信: {text}')

    # ---- 手動操縦 -------------------------------------------------------
    def set_manual(self, lin, ang):
        """GUI のボタン/キー押下時に呼ばれる。"""
        # 手動入力は常に優先: 自動巡回から抜ける。
        if self._auto:
            self.set_auto(False)
        self._manual = (lin, ang)
        self._manual_active = (lin != 0.0 or ang != 0.0)

    def stop_manual(self):
        self._manual = (0.0, 0.0)
        self._manual_active = False
        self._cmd_pub.publish(Twist())  # 即時停止

    # ---- 原点ワープ -----------------------------------------------------
    def warp_to_origin(self):
        """ロボットを map 原点へワープさせ、AMCL を再初期化する。

        ロボットが隅へ突っ込んでスタックしたときに有用。まず動きを止め、Gazebo
        モデルを (0,0) へ移動し、その後 /initialpose を publish して AMCL の推定を
        新しい姿勢に合わせる（そうしないと自己位置が古い場所に残り Nav2 が誤計画する）。
        """
        # まず自動巡回を抜けて車輪を止める。
        if self._auto:
            self.set_auto(False)
        self.stop_manual()

        # Gazebo エンティティを原点へ移動（Gazebo Classic の CLI）。
        try:
            subprocess.run(
                ['gz', 'model', '-m', ROBOT_ENTITY,
                 '-x', '0.0', '-y', '0.0', '-z', '0.05', '-Y', '0.0'],
                timeout=5.0, check=False)
        except Exception as e:  # pragma: no cover
            self.get_logger().warn(f'warp: gz model failed: {e}')

        # AMCL を原点で再初期化（確実に反映されるよう数回 publish する）。
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose.orientation.w = 1.0
        cov = [0.0] * 36
        cov[0] = 0.25   # x
        cov[7] = 0.25   # y
        cov[35] = 0.068  # yaw
        msg.pose.covariance = cov
        for _ in range(5):
            msg.header.stamp = self.get_clock().now().to_msg()
            self._initpose_pub.publish(msg)
            time.sleep(0.1)
        self.get_logger().info('warped robot to origin and re-seeded AMCL')

    def _tick(self):
        if self._manual_active:
            t = Twist()
            t.linear.x, t.angular.z = self._manual
            self._cmd_pub.publish(t)

    # ---- 自動巡回（Nav2 ウェイポイント巡回）------------------------------
    def set_auto(self, on):
        self._auto = on
        if on:
            self._manual_active = False
            self._send_next_waypoint()
        else:
            self._cancel_nav()

    def _send_next_waypoint(self):
        if not self._auto or self._nav_busy:
            return
        if not self._nav_client.server_is_ready():
            # Nav2 がまだ起動していない。ウォッチドッグが再試行する。
            return
        x, y = PATROL_WAYPOINTS[self._wp_index]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.w = 1.0
        self._nav_busy = True
        self._goal_sent_at = time.monotonic()
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)
        self.get_logger().info(
            f'auto-explore -> waypoint {self._wp_index}: ({x:.1f}, {y:.1f})')

    def _auto_watchdog(self):
        if not self._auto:
            return
        # ロボットを常にどこかへ向かわせ続ける: ゴールが飛んでいなければ
        # （Nav2 の起動が遅れた、ゴールが失われた等）1つ送る。
        if not self._nav_busy:
            self._send_next_waypoint()
            return
        # 時間がかかりすぎているウェイポイント（隅にハマったロボット）は諦めて
        # 次へ進む。1箇所のスタックが巡回全体を止めないようにするため。
        if (self._goal_sent_at is not None
                and time.monotonic() - self._goal_sent_at > WAYPOINT_TIMEOUT_S):
            self.get_logger().warn(
                f'waypoint {self._wp_index} timed out ({WAYPOINT_TIMEOUT_S:.0f}s); '
                'skipping to next')
            self._cancel_nav()
            self._advance_waypoint()

    def _on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Nav2 rejected goal; advancing waypoint')
            self._advance_waypoint()
            return
        self._nav_goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        self._nav_busy = False
        self._nav_goal_handle = None
        if self._auto:
            # 到達しても中断しても次の部屋へ進み、巡回を流し続ける
            # （status はデバッグ用にログ出力する）。
            status = future.result().status
            ok = status == GoalStatus.STATUS_SUCCEEDED
            self.get_logger().info(
                f'waypoint {self._wp_index} {"reached" if ok else "ended"} '
                f'(status {status})')
            self._advance_waypoint()

    def _advance_waypoint(self):
        self._wp_index = (self._wp_index + 1) % len(PATROL_WAYPOINTS)
        if self._auto:
            self._send_next_waypoint()

    def _cancel_nav(self):
        if self._nav_goal_handle is not None:
            self._nav_goal_handle.cancel_goal_async()
            self._nav_goal_handle = None
        self._nav_busy = False
        self._cmd_pub.publish(Twist())  # 確実に停止させる


class TeleopGui:
    """tkinter のウィンドウ。メインスレッドで動作し、rclpy は別スレッドで spin する。"""

    def __init__(self, node: TeleopGuiNode):
        self.node = node
        self.root = tk.Tk()
        self.root.title('TB3 Teleop / 自動巡回')
        self.root.configure(bg='#202028')

        # --- 自動巡回トグル（大きく ON/OFF が分かるボタン）---
        self.auto_btn = tk.Button(
            self.root, text='自動巡回: OFF', width=24, height=2,
            font=('Sans', 13, 'bold'), bg='#553333', fg='white',
            activebackground='#664444', command=self._toggle_auto)
        self.auto_btn.grid(row=0, column=0, columnspan=3, padx=8, pady=(10, 4))

        hint = tk.Label(
            self.root, bg='#202028', fg='#aaaaaa', font=('Sans', 9),
            text='矢印（またはテンキー 8/2/4/6）を押している間だけ走行。\n'
                 '自動巡回 ON = Nav2 で部屋を巡回。')
        hint.grid(row=1, column=0, columnspan=3, pady=(0, 6))

        # --- 矢印ボタン（押している間だけ動く）---
        mk = self._make_hold_button
        mk('▲\n前進', LINEAR_SPEED, 0.0, row=2, column=1)
        mk('◀\n左旋回', 0.0, ANGULAR_SPEED, row=3, column=0)
        mk('▼\n後退', -LINEAR_SPEED, 0.0, row=3, column=1)
        mk('▶\n右旋回', 0.0, -ANGULAR_SPEED, row=3, column=2)

        # --- 原点ワープ（スタックしたロボットの救済）---
        self.warp_btn = tk.Button(
            self.root, text='⌂ 原点へワープ', width=24, height=2,
            font=('Sans', 11, 'bold'), bg='#445588', fg='white',
            activebackground='#5566aa', command=self._warp)
        self.warp_btn.grid(row=4, column=0, columnspan=3, padx=8, pady=(8, 4))

        # --- セマンティック物体メモリへのクエリ（semantic_memory:=True 起動時のみ機能）---
        # 入力欄に「椅子」「人」等を打って送信すると、semantic_query_node が記憶から
        # 物体座標を引き、その手前へ Nav2 で移動する。
        query_label = tk.Label(
            self.root, bg='#202028', fg='#aaaaaa', font=('Sans', 9),
            text='物体へ行く（要 semantic_memory:=True）: 例「人」「車」')
        query_label.grid(row=5, column=0, columnspan=3, pady=(6, 0))
        self.query_entry = tk.Entry(
            self.root, width=16, font=('Sans', 11),
            bg='#303040', fg='white', insertbackground='white')
        self.query_entry.grid(row=6, column=0, columnspan=2, padx=(8, 4), pady=(2, 4))
        self.query_entry.bind('<Return>', lambda e: self._send_query())
        self.query_btn = tk.Button(
            self.root, text='探して移動', width=8, height=1,
            font=('Sans', 10, 'bold'), bg='#446644', fg='white',
            activebackground='#557755', command=self._send_query)
        self.query_btn.grid(row=6, column=2, padx=(4, 8), pady=(2, 4))

        # --- 追従/探索（object_seeker。semantic_memory:=True 起動時のみ機能）---
        # 入力欄のクラスを「追って」=動く対象を追従 / 「探して」=巡回して見つけたら接近。
        seek_label = tk.Label(
            self.root, bg='#202028', fg='#aaaaaa', font=('Sans', 9),
            text='追従/探索: クラスを入れて［追って］か［探して］')
        seek_label.grid(row=7, column=0, columnspan=3, pady=(4, 0))
        self.seek_entry = tk.Entry(
            self.root, width=10, font=('Sans', 11),
            bg='#303040', fg='white', insertbackground='white')
        self.seek_entry.grid(row=8, column=0, padx=(8, 2), pady=(2, 10))
        self.follow_btn = tk.Button(
            self.root, text='追って', width=6, height=1,
            font=('Sans', 10, 'bold'), bg='#664422', fg='white',
            activebackground='#775533',
            command=lambda: self._send_seek('追って'))
        self.follow_btn.grid(row=8, column=1, padx=2, pady=(2, 10))
        self.search_btn = tk.Button(
            self.root, text='探して', width=6, height=1,
            font=('Sans', 10, 'bold'), bg='#224466', fg='white',
            activebackground='#335577',
            command=lambda: self._send_seek('探して'))
        self.search_btn.grid(row=8, column=2, padx=(2, 8), pady=(2, 10))

        # --- キーボード: 矢印キー + テンキー ---
        binds = {
            'forward': ('<KeyPress-Up>', '<KeyPress-KP_Up>', '<KeyPress-KP_8>'),
            'back': ('<KeyPress-Down>', '<KeyPress-KP_Down>', '<KeyPress-KP_2>'),
            'left': ('<KeyPress-Left>', '<KeyPress-KP_Left>', '<KeyPress-KP_4>'),
            'right': ('<KeyPress-Right>', '<KeyPress-KP_Right>', '<KeyPress-KP_6>'),
        }
        cmds = {
            'forward': (LINEAR_SPEED, 0.0), 'back': (-LINEAR_SPEED, 0.0),
            'left': (0.0, ANGULAR_SPEED), 'right': (0.0, -ANGULAR_SPEED),
        }
        for name, keys in binds.items():
            lin, ang = cmds[name]
            for k in keys:
                self.root.bind(k, lambda e, ln=lin, an=ang: self.node.set_manual(ln, an))
            # キー解放（対応する解放イベントのいずれか）で停止する
            for k in keys:
                kr = k.replace('KeyPress', 'KeyRelease')
                self.root.bind(kr, lambda e: self.node.stop_manual())

        self.root.protocol('WM_DELETE_WINDOW', self._on_close)
        # Ctrl-C / ノード終了でウィンドウを閉じられるよう定期ポーリングする。
        self.root.after(200, self._poll)

    def _send_query(self):
        self.node.send_query(self.query_entry.get())

    def _send_seek(self, verb):
        """seek_entry のクラスに動詞（追って/探して）を付けて object_seeker へ送る。"""
        cls = self.seek_entry.get().strip()
        if cls:
            self.node.send_seek(f'{cls}を{verb}')

    def _make_hold_button(self, label, lin, ang, row, column):
        b = tk.Button(self.root, text=label, width=8, height=2,
                      font=('Sans', 11, 'bold'), bg='#334455', fg='white',
                      activebackground='#445566')
        b.grid(row=row, column=column, padx=4, pady=4)
        b.bind('<ButtonPress-1>', lambda e: self.node.set_manual(lin, ang))
        b.bind('<ButtonRelease-1>', lambda e: self.node.stop_manual())
        return b

    def _toggle_auto(self):
        new = not self.node._auto
        self.node.set_auto(new)
        if new:
            self.auto_btn.config(text='自動巡回: ON', bg='#2e8b57',
                                 activebackground='#3fa169')
        else:
            self.auto_btn.config(text='自動巡回: OFF', bg='#553333',
                                 activebackground='#664444')

    def _warp(self):
        # 別スレッドで実行する: warp_to_origin はブロックする（gz サブプロセス +
        # initialpose の sleep）ため、そのまま呼ぶと GUI がフリーズする。ワープは
        # 自動巡回をキャンセルするので、トグル表示も OFF に反映する。
        self.auto_btn.config(text='自動巡回: OFF', bg='#553333',
                             activebackground='#664444')
        self.warp_btn.config(text='⌂ ワープ中...', state='disabled')

        def run():
            self.node.warp_to_origin()
            self.warp_btn.config(text='⌂ 原点へワープ', state='normal')

        threading.Thread(target=run, daemon=True).start()

    def _poll(self):
        if not rclpy.ok():
            self.root.destroy()
            return
        # 自動巡回が自分で OFF になった（手動入力）場合、トグル表示を同期させる。
        if not self.node._auto and self.auto_btn.cget('text').endswith('ON'):
            self.auto_btn.config(text='自動巡回: OFF', bg='#553333',
                                 activebackground='#664444')
        self.root.after(200, self._poll)

    def _on_close(self):
        self.node.stop_manual()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopGuiNode()
    if tk is None:
        node.get_logger().error('tkinter not available; GUI cannot start')
        rclpy.shutdown()
        return

    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    gui = TeleopGui(node)
    try:
        gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_manual()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
