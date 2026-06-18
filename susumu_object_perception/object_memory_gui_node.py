#!/usr/bin/env python3
"""記憶物体の一覧 GUI。「どこに何があるか」を表で見て、行クリックで詳細を出す。

object_memory_node が書く SQLite DB（既定 ~/.ros/object_memory.sqlite3）を定期的に
読み込み、tkinter Treeview に一覧表示する。各行 = 記憶している 1 物体:
クラス / map 座標 / existence / hits / 最終観測からの経過秒。

  * 行をクリックすると下部の詳細パネルに全フィールドを表示する。
  * 「選択物体へ行く」ボタンで、選択物体のクラスを /semantic_query に投げて Nav2 移動
    させる（semantic_query_node が起動していれば移動する）。

teleop_gui_node.py と同じ作法: UI は tkinter（メインスレッド）、rclpy は別スレッドで spin。
ヘッドレス環境では tk import に失敗するのでエラーログを出して終了する。

各種 launch で semantic_memory:=True のとき一緒に起動される。DB を読むだけなので
object_memory より後に起動していなくても（DB が空でも）害は無い。
"""

import os
import sqlite3
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import tkinter as tk
    from tkinter import ttk
except Exception:  # pragma: no cover - ヘッドレス環境向けガード
    tk = None
    ttk = None


class MemoryGuiNode(Node):
    """rclpy 側。/semantic_query への publish と現在時刻の供給のみ持つ軽量ノード。"""

    def __init__(self):
        super().__init__('object_memory_gui')
        self.declare_parameter(
            'db_path', os.path.expanduser('~/.ros/object_memory.sqlite3'))
        self.declare_parameter('refresh_hz', 1.0)
        self.db_path = self.get_parameter('db_path').value
        self.refresh_hz = float(self.get_parameter('refresh_hz').value)
        self._query_pub = self.create_publisher(String, '/semantic_query', 10)
        self.get_logger().info(
            f'object_memory_gui started. DB({self.db_path}) を一覧表示')

    def send_goto(self, class_name):
        if class_name:
            self._query_pub.publish(String(data=class_name))
            self.get_logger().info(f'「{class_name}」へ移動を要求')

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9


def read_objects(db_path):
    """DB から記憶物体を読む。DB が無い/読めない間は空リスト（GUI は落とさない）。"""
    if not os.path.exists(db_path):
        return []
    try:
        db = sqlite3.connect(f'file:{db_path}?mode=ro', uri=True)
    except sqlite3.Error:
        return []
    try:
        rows = db.execute(
            "SELECT id, class_name, x, y, z, size_x, size_y, size_z, "
            "existence, hits, last_seen FROM objects ORDER BY existence DESC"
        ).fetchall()
    except sqlite3.Error:
        rows = []
    finally:
        db.close()
    cols = ['id', 'class_name', 'x', 'y', 'z', 'size_x', 'size_y', 'size_z',
            'existence', 'hits', 'last_seen']
    return [dict(zip(cols, r)) for r in rows]


class MemoryGui:
    """tkinter ウィンドウ。一覧 Treeview + 詳細パネル + 移動ボタン。"""

    def __init__(self, node: MemoryGuiNode):
        self.node = node
        self.root = tk.Tk()
        self.root.title('記憶物体 一覧')
        self.root.configure(bg='#202028')
        self.root.geometry('560x420')
        self._selected = None  # 選択中の物体 dict

        title = tk.Label(
            self.root, text='記憶している物体（どこに何があるか）',
            bg='#202028', fg='white', font=('Sans', 12, 'bold'))
        title.pack(pady=(8, 4))

        # --- 一覧 Treeview ---
        cols = ('id', 'class', 'x', 'y', 'exist', 'hits', 'seen')
        headers = {'id': '#', 'class': 'クラス', 'x': 'X[m]', 'y': 'Y[m]',
                   'exist': '確信度', 'hits': '観測', 'seen': '最終[s前]'}
        widths = {'id': 36, 'class': 130, 'x': 70, 'y': 70, 'exist': 64,
                  'hits': 56, 'seen': 80}
        style = ttk.Style()
        style.theme_use('default')
        style.configure('Treeview', background='#2b2b36',
                        fieldbackground='#2b2b36', foreground='white',
                        rowheight=22)
        style.configure('Treeview.Heading', background='#3a3a48',
                        foreground='white')
        self.tree = ttk.Treeview(
            self.root, columns=cols, show='headings', height=10)
        for c in cols:
            self.tree.heading(c, text=headers[c])
            self.tree.column(c, width=widths[c], anchor='center')
        self.tree.pack(fill='both', expand=True, padx=8)
        self.tree.bind('<<TreeviewSelect>>', self._on_select)

        # --- 詳細パネル ---
        self.detail = tk.Label(
            self.root, text='行をクリックすると詳細を表示',
            bg='#202028', fg='#cccccc', font=('Sans', 10), justify='left',
            anchor='w')
        self.detail.pack(fill='x', padx=8, pady=(6, 2))

        # --- 移動ボタン ---
        self.goto_btn = tk.Button(
            self.root, text='選択物体へ行く', width=18, height=1,
            font=('Sans', 11, 'bold'), bg='#446644', fg='white',
            activebackground='#557755', command=self._on_goto, state='disabled')
        self.goto_btn.pack(pady=(2, 10))

        self.root.protocol('WM_DELETE_WINDOW', self._on_close)
        self._iid_to_obj = {}
        self._refresh()
        self.root.after(200, self._poll)

    def _refresh(self):
        """DB を読んで Treeview を作り直す。選択は id で復元する。"""
        objs = read_objects(self.node.db_path)
        now = self.node.now_sec()
        prev_id = self._selected['id'] if self._selected else None

        self.tree.delete(*self.tree.get_children())
        self._iid_to_obj = {}
        reselect_iid = None
        for o in objs:
            seen_ago = max(0.0, now - o['last_seen']) if o['last_seen'] else 0.0
            iid = self.tree.insert('', 'end', values=(
                o['id'], o['class_name'],
                f"{o['x']:.2f}", f"{o['y']:.2f}",
                f"{o['existence']:.2f}", o['hits'], f"{seen_ago:.1f}"))
            self._iid_to_obj[iid] = o
            if o['id'] == prev_id:
                reselect_iid = iid

        if reselect_iid:
            self.tree.selection_set(reselect_iid)
        elif prev_id is not None:
            # 選択物体が消えた（忘却された）。
            self._selected = None
            self.goto_btn.config(state='disabled')
            self.detail.config(text='（選択物体は記憶から消えました）')

        # 周期更新。
        period = int(1000 / max(0.2, self.node.refresh_hz))
        self.root.after(period, self._refresh)

    def _on_select(self, _event):
        sel = self.tree.selection()
        if not sel:
            return
        o = self._iid_to_obj.get(sel[0])
        if o is None:
            return
        self._selected = o
        self.goto_btn.config(state='normal')
        self.detail.config(text=(
            f"#{o['id']}  クラス: {o['class_name']}\n"
            f"map 座標: ({o['x']:.2f}, {o['y']:.2f}, {o['z']:.2f})   "
            f"サイズ: {o['size_x']:.2f}×{o['size_y']:.2f}×{o['size_z']:.2f} m\n"
            f"確信度(existence): {o['existence']:.3f}   観測回数: {o['hits']}"))

    def _on_goto(self):
        if self._selected:
            self.node.send_goto(self._selected['class_name'])
            self.detail.config(
                text=f"「{self._selected['class_name']}」へ移動を要求しました")

    def _poll(self):
        if not rclpy.ok():
            self.root.destroy()
            return
        self.root.after(200, self._poll)

    def _on_close(self):
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = MemoryGuiNode()
    if tk is None:
        node.get_logger().error(
            'tkinter が import できません（ヘッドレス環境）。GUI を起動できないため終了します。')
        node.destroy_node()
        rclpy.shutdown()
        return

    # rclpy は別スレッドで spin、tkinter はメインスレッドで回す（teleop_gui と同作法）。
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        gui = MemoryGui(node)
        gui.run()
    except Exception as e:  # pragma: no cover
        node.get_logger().error(f'GUI 異常終了: {e}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
