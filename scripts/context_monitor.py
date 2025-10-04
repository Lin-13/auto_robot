import grpc
import sys
import numpy as np

sys.path.append("proto")
import proto.monitor_pb2 as monitor_pb2
import proto.monitor_pb2_grpc as monitor_pb2_grpc
import time
import threading
from utils import draw_coordinate_system
import matplotlib.pyplot as plt


class MonitorClient:
    def __init__(self, server_address="localhost:50051"):
        """初始化客户端并连接到gRPC服务器"""
        self.channel = grpc.insecure_channel(server_address)
        self.stub = monitor_pb2_grpc.MonitorServiceStub(self.channel)
        self.buffer = {}

    def get_variable(self, variable_name):
        """获取指定变量的值"""
        try:
            request = monitor_pb2.VariableRequest(name=variable_name)
            return self.stub.GetVariable(request)
        except grpc.RpcError as e:
            print(f"RPC错误: {e.details()}")
            return None

    def set_variable(self, variable_name, variable_value):
        """设置指定变量的值"""
        try:
            request = monitor_pb2.VariableUpdate(
                name=variable_name, value=variable_value
            )
            return self.stub.SetVariable(request)
        except grpc.RpcError as e:
            print(f"RPC错误: {e.details()}")
            return None

    def list_variables(self):
        """列出所有可用变量"""
        try:
            request = monitor_pb2.EmptyRequest()
            response = self.stub.ListVariables(request)
            return response.names
        except grpc.RpcError as e:
            print(f"RPC错误: {e.details()}")
            return None

    def client_str_to_numpy(self, str_value):
        """将客户端接收到的字符串转换为numpy数组"""
        try:
            # 尝试将字符串解析为列表
            rows = str_value.strip("[]").split(";")
            matrix_list = []
            for row in rows:
                # 处理每行：去除空格并转换为浮点数列表
                elements = list(map(float, row.replace(" ", "").split(",")))
                matrix_list.append(elements)
            # 转换为numpy数组
            numpy_matrix = np.array(matrix_list)
            return numpy_matrix
        except (ValueError, SyntaxError):
            return None

    def run(self):
        """通过子线程每隔一段时间更新变量值并放入buffer"""

        def update_buffer(ax, plot_matrix=False):
            plt.ion()
            fig = ax.figure
            plt.show(block=False)
            while True:
                variables = self.list_variables()
                matrix = {}
                if variables:
                    for var_name in variables:
                        response = self.get_variable(var_name)
                        if response:
                            self.buffer[var_name] = response.value
                            if self.client_str_to_numpy(response.value) is not None:
                                matrix[var_name] = self.client_str_to_numpy(
                                    response.value
                                )
                        else:
                            print(f"获取变量 {var_name} 失败")
                if plot_matrix:
                    ax.clear()
                    T = np.array(
                        [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
                    )
                    draw_coordinate_system(ax, T, length=1.0, label="O")
                    for var_name, mat in matrix.items():
                        if mat.shape == (4, 4):
                            print(f"变量 {var_name} 的矩阵: \n{mat}\n")
                            draw_coordinate_system(ax, mat, length=0.3, label=var_name)
                    # 设置坐标轴范围和标签
                    ax.set_xlim(-1.5, 1.5)
                    ax.set_ylim(-1.5, 1.5)
                    ax.set_zlim(-0, 3)
                    ax.set_xlabel("X")
                    ax.set_ylabel("Y")
                    ax.set_zlabel("Z")
                    fig.canvas.draw_idle()
                    fig.canvas.flush_events()
                time.sleep(0.05)

        fig = plt.figure(figsize=(8, 6))
        ax = fig.add_subplot(111, projection="3d")

        update_buffer(ax, plot_matrix=True)

    def close(self):
        """关闭连接"""
        self.channel.close()


def run_monitor_client():
    # 创建客户端实例
    client = MonitorClient("localhost:50051")

    try:
        while True:
            command = input("\n命令 (get <变量名>/set <变量名> <值>/list/exit): ")
            command = command.strip()

            if command == "exit":
                print("退出客户端...")
                break

            elif command == "list":
                variables = client.list_variables()
                if variables:
                    print("可用变量:")
                    print(", ".join(variables))
                else:
                    print("获取变量列表失败或没有可用变量")

            elif command.startswith("get "):
                parts = command.split(maxsplit=1)
                if len(parts) < 2:
                    print("无效的get命令。用法: get <变量名>")
                    continue
                var_name = parts[1]

                response = client.get_variable(var_name)
                if response:
                    if response.found:
                        print(f"{response.name} ({response.type}): {response.value}")
                    else:
                        print(f"变量 {var_name} 未找到")

            elif command.startswith("set "):
                parts = command.split(maxsplit=2)
                if len(parts) < 3:
                    print("无效的set命令。用法: set <变量名> <值>")
                    continue
                var_name = parts[1]
                value = parts[2]

                response = client.set_variable(var_name, value)
                if response:
                    if response.success:
                        print(f"成功设置变量 {var_name} 为 {value}")
                    else:
                        print(f"设置失败: {response.message}")

            else:
                print("未知命令。可用命令: get, set, list, exit")

    finally:
        client.close()


if __name__ == "__main__":
    # run_monitor_client()
    client = MonitorClient("localhost:50051")
    client.run()
