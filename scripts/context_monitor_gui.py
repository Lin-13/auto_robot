import grpc
import sys
import numpy as np
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import threading
import time
import logging  # 新增日志模块

# 配置日志输出
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)

sys.path.append("proto")
import proto.monitor_pb2 as monitor_pb2
import proto.monitor_pb2_grpc as monitor_pb2_grpc
from utils import draw_coordinate_system, draw_vector


class MonitorClient:
    def __init__(self, server_address="localhost:50051"):
        self.channel = grpc.insecure_channel(server_address)
        self.stub = monitor_pb2_grpc.MonitorServiceStub(self.channel)
        self.buffer = {}
        self.variables = []  # 标准Python列表
        self.update_callback = None

    def get_variable(self, variable_name):
        try:
            request = monitor_pb2.VariableRequest(name=variable_name)
            logging.info(f"发送获取变量请求: {variable_name}")
            response = self.stub.GetVariable(request)
            logging.info(f"获取变量响应: {response}")
            return response
        except grpc.RpcError as e:
            logging.error(f"获取变量RPC错误: {e.code()} - {e.details()}")
            return None

    def set_variable(self, variable_name, variable_value):
        try:
            # 构造请求并打印日志
            request = monitor_pb2.VariableUpdate(
                name=variable_name, value=variable_value
            )
            logging.info(
                f"发送设置变量请求: 变量名={variable_name}, 值={variable_value}"
            )

            # 发送请求并获取响应
            response = self.stub.SetVariable(request)
            logging.info(f"设置变量响应: {response}")  # 打印完整响应
            return response
        except grpc.RpcError as e:
            logging.error(f"设置变量RPC错误: {e.code()} - {e.details()}")
            return None

    def list_variables(self):
        try:
            request = monitor_pb2.EmptyRequest()
            response = self.stub.ListVariables(request)
            self.variables = list(response.names)  # 转为标准列表
            logging.info(f"获取变量列表: {self.variables}")
            return self.variables
        except grpc.RpcError as e:
            logging.error(f"获取变量列表RPC错误: {e.code()} - {e.details()}")
            return None

    def client_str_to_numpy(self, str_value):
        try:
            rows = str_value.strip("[]").split(";")
            matrix_list = []
            for row in rows:
                elements = list(map(float, row.replace(" ", "").split(",")))
                matrix_list.append(elements)
            return np.array(matrix_list)
        except (ValueError, SyntaxError) as e:
            # logging.warning(f"解析值为矩阵失败: {e}")
            return None

    def start_monitoring(self, t=1):
        def monitor_loop(t):
            while True:
                self.list_variables()
                for var_name in self.variables:
                    response = self.get_variable(var_name)
                    if response and response.found:
                        self.buffer[var_name] = response.value
                if self.update_callback:
                    self.update_callback()
                time.sleep(t)

        self.monitor_thread = threading.Thread(
            target=monitor_loop, daemon=True, args=(t,)
        )
        self.monitor_thread.start()

    def close(self):
        self.channel.close()


class MonitorGUI:
    def __init__(self, root, client):
        self.root = root
        self.root.title("Monitor Client")
        self.root.geometry("1400x800")
        self.client = client
        self.client.update_callback = self.update_ui

        # 创建主布局
        self.notebook = ttk.Notebook(root)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # 变量管理标签页
        self.var_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.var_frame, text="变量管理")

        # 3D可视化标签页
        self.visual_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.visual_frame, text="3D可视化")

        # 初始化界面
        self.init_var_management()
        self.init_3d_visualization()

        # 启动监控
        self.client.start_monitoring()

    def init_var_management(self):
        # 左侧变量列表
        left_frame = ttk.Frame(self.var_frame)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)

        ttk.Label(left_frame, text="变量列表:").pack(anchor=tk.W, padx=5, pady=5)

        self.var_listbox = tk.Listbox(
            left_frame, selectmode=tk.SINGLE, width=30, height=20
        )
        self.var_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        self.var_listbox.bind("<<ListboxSelect>>", self.on_var_select)

        scrollbar = ttk.Scrollbar(
            left_frame, orient=tk.VERTICAL, command=self.var_listbox.yview
        )
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.var_listbox.config(yscrollcommand=scrollbar.set)

        # 右侧变量详情和操作区
        right_frame = ttk.Frame(self.var_frame)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5, pady=5)

        # 变量详情
        ttk.Label(right_frame, text="变量详情:").pack(anchor=tk.W, padx=5, pady=5)

        self.var_detail = scrolledtext.ScrolledText(right_frame, width=50, height=10)
        self.var_detail.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        self.var_detail.config(state=tk.DISABLED)

        # 操作区（新增格式提示）
        operation_frame = ttk.LabelFrame(
            right_frame,
            text="操作（矩阵格式示例: [[1,0,0,0];[0,1,0,0];[0,0,1,0];[0,0,0,1]]）",
        )
        operation_frame.pack(fill=tk.X, padx=5, pady=5)

        # Get操作
        ttk.Label(operation_frame, text="变量名:").grid(
            row=0, column=0, padx=5, pady=5, sticky=tk.W
        )
        self.get_var_entry = ttk.Entry(operation_frame, width=30)
        self.get_var_entry.grid(row=0, column=1, padx=5, pady=5)
        ttk.Button(operation_frame, text="获取", command=self.get_variable).grid(
            row=0, column=2, padx=5, pady=5
        )

        # Set操作
        ttk.Label(operation_frame, text="设置值:").grid(
            row=1, column=0, padx=5, pady=5, sticky=tk.W
        )
        self.set_val_entry = ttk.Entry(operation_frame, width=30)
        self.set_val_entry.grid(row=1, column=1, padx=5, pady=5)
        ttk.Button(operation_frame, text="设置", command=self.set_variable).grid(
            row=1, column=2, padx=5, pady=5
        )

        # 刷新按钮
        ttk.Button(
            operation_frame, text="刷新列表", command=self.refresh_var_list
        ).grid(row=2, column=0, columnspan=3, pady=5)

    def init_3d_visualization(self):
        self.fig = Figure(figsize=(8, 6), dpi=100)
        self.ax = self.fig.add_subplot(111, projection="3d")

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.visual_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self.start_3d_refresh()

    def start_3d_refresh(self):
        def refresh_loop():
            while True:
                self.update_3d_visualization()
                time.sleep(0.1)

        self.refresh_thread = threading.Thread(target=refresh_loop, daemon=True)
        self.refresh_thread.start()

    def update_3d_visualization(self):
        self.ax.clear()
        T = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        draw_coordinate_system(self.ax, T, length=1.0, label="O")

        for var_name, value in self.client.buffer.items():
            mat = self.client.client_str_to_numpy(value)
            if mat is not None:
                if mat.shape == (4, 4):
                    draw_coordinate_system(self.ax, mat, length=0.3, label=var_name)
                    # logging.info(f"Draw coordinate system {var_name}")
                elif mat.shape == (3, 3):
                    draw_vector(
                        self.ax, T[:3, 3], mat[:, 0], color="r", label=var_name + "X"
                    )
                    draw_vector(
                        self.ax, T[:3, 3], mat[:, 1], color="g", label=var_name + "Y"
                    )
                    draw_vector(
                        self.ax, T[:3, 3], mat[:, 2], color="b", label=var_name + "Z"
                    )
                    # logging.info(f"Draw vector {var_name}X")
                elif mat.shape == (3, 1) or mat.shape == (1, 3):
                    draw_vector(self.ax, T[:3, 3], mat[:, 0], color="k", label=var_name)
                    # logging.info(f"Draw vector {var_name}P")

        self.ax.set_xlim(-1.5, 1.5)
        self.ax.set_ylim(-1.5, 1.5)
        self.ax.set_zlim(-0, 3)
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")
        # self.ax.equal()
        self.canvas.draw_idle()

    def update_ui(self):
        self.update_var_list()
        selected = self.var_listbox.curselection()
        if selected:
            var_name = self.var_listbox.get(selected[0])
            self.update_var_detail(var_name)

    def update_var_list(self):
        current_selection = self.var_listbox.curselection()
        current_var = (
            self.var_listbox.get(current_selection[0]) if current_selection else None
        )

        self.var_listbox.delete(0, tk.END)
        for var in self.client.variables:
            self.var_listbox.insert(tk.END, var)
            if var == current_var:
                idx = self.client.variables.index(var)
                self.var_listbox.selection_set(idx)
                self.var_listbox.see(idx)

    def on_var_select(self, event):
        selected = self.var_listbox.curselection()
        if selected:
            var_name = self.var_listbox.get(selected[0])
            self.update_var_detail(var_name)
            self.get_var_entry.delete(0, tk.END)
            self.get_var_entry.insert(0, var_name)

    def update_var_detail(self, var_name):
        if var_name in self.client.buffer:
            value = self.client.buffer[var_name]
            self.var_detail.config(state=tk.NORMAL)
            self.var_detail.delete(1.0, tk.END)
            self.var_detail.insert(tk.END, f"变量名: {var_name}\n\n")
            self.var_detail.insert(tk.END, f"值:\n{value}\n\n")

            mat = self.client.client_str_to_numpy(value)
            if mat is not None:
                self.var_detail.insert(tk.END, f"矩阵形式:\n{mat}")

            self.var_detail.config(state=tk.DISABLED)

    def get_variable(self):
        var_name = self.get_var_entry.get().strip()
        if not var_name:
            messagebox.showerror("错误", "请输入变量名")
            return

        response = self.client.get_variable(var_name)
        if response:
            if response.found:
                self.var_detail.config(state=tk.NORMAL)
                self.var_detail.delete(1.0, tk.END)
                self.var_detail.insert(tk.END, f"变量名: {response.name}\n\n")
                self.var_detail.insert(tk.END, f"类型: {response.type}\n\n")
                self.var_detail.insert(tk.END, f"值:\n{response.value}")
                self.var_detail.config(state=tk.DISABLED)
            else:
                messagebox.showerror("错误", f"变量 {var_name} 未找到")
        else:
            messagebox.showerror(
                "错误", f"获取变量 {var_name} 失败（查看日志获取详情）"
            )

    def set_variable(self):
        var_name = self.get_var_entry.get().strip()
        value = self.set_val_entry.get().strip()

        if not var_name or not value:
            messagebox.showerror("错误", "请输入变量名和值")
            return

        # 检查值是否符合矩阵格式（如果服务端要求）
        if "[" in value and "]" in value:  # 简单判断是否为矩阵
            try:
                # 尝试解析为矩阵，验证格式
                self.client.client_str_to_numpy(value)
            except Exception as e:
                messagebox.showerror(
                    "格式错误",
                    f"值格式不正确（矩阵格式示例: [[1,0,0,0];[0,1,0,0];[0,0,1,0];[0,0,0,1]]）\n错误: {e}",
                )
                return

        # 发送设置请求
        response = self.client.set_variable(var_name, value)

        if not response:
            messagebox.showerror(
                "错误", "未收到服务端响应（可能网络问题或服务端未启动）"
            )
            return

        if response.success:
            # messagebox.showinfo("成功", f"成功设置变量 {var_name} 为 {value}")
            self.set_val_entry.delete(0, tk.END)
            self.get_variable()  # 立即刷新显示
        else:
            # 显示服务端返回的具体失败原因
            messagebox.showerror(
                "设置失败",
                f"服务端拒绝设置：{response.message if response.message else '未知原因'}",
            )

    def refresh_var_list(self):
        self.client.list_variables()
        self.update_var_list()


def main():
    root = tk.Tk()
    client = MonitorClient("localhost:50051")
    app = MonitorGUI(root, client)

    def on_close():
        client.close()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
