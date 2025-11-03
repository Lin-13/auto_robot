import re
import matplotlib.pyplot as plt
import numpy as np
from collections import defaultdict


class OptiTrackVisualizer:
    def __init__(self, filename="optitrack_target.txt"):
        self.filename = filename
        self.data = defaultdict(
            lambda: {
                "timestamps": [],
                "positions": {"x": [], "y": [], "z": []},
                "quaternions": {"w": [], "x": [], "y": [], "z": []},
            }
        )
        self.fig = None
        self.axes = None

    def parse_data(self):
        """解析optitrack_target.txt文件"""
        try:
            with open(self.filename, "r", encoding="utf-8") as f:
                content = f.read()
        except FileNotFoundError:
            print(f"文件 {self.filename} 未找到！")
            return False

        # 正则表达式匹配数据块
        pattern = r"\[刚体 (.*?)\].*?时间戳: (\d+)\.(\d+).*?位置: X=([-\d\.]+)m, Y=([-\d\.]+)m, Z=([-\d\.]+)m.*?姿态: w=([-\d\.]+), x=([-\d\.]+), y=([-\d\.]+), z=([-\d\.]+)"

        matches = re.findall(pattern, content, re.DOTALL)

        for match in matches:
            rb_name = match[0]
            timestamp = float(match[1]) + float(match[2]) / 1000000.0
            pos_x, pos_y, pos_z = float(match[3]), float(match[4]), float(match[5])
            quat_w, quat_x, quat_y, quat_z = (
                float(match[6]),
                float(match[7]),
                float(match[8]),
                float(match[9]),
            )

            self.data[rb_name]["timestamps"].append(timestamp)
            self.data[rb_name]["positions"]["x"].append(pos_x)
            self.data[rb_name]["positions"]["y"].append(pos_y)
            self.data[rb_name]["positions"]["z"].append(pos_z)
            self.data[rb_name]["quaternions"]["w"].append(quat_w)
            self.data[rb_name]["quaternions"]["x"].append(quat_x)
            self.data[rb_name]["quaternions"]["y"].append(quat_y)
            self.data[rb_name]["quaternions"]["z"].append(quat_z)

        print(f"解析完成！找到 {len(self.data)} 个刚体的数据")
        for rb_name, rb_data in self.data.items():
            print(f"  - {rb_name}: {len(rb_data['timestamps'])} 个数据点")

        return len(self.data) > 0

    def plot_3d_trajectory(self):
        """绘制3D轨迹图"""
        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(111, projection="3d")

        colors = ["red", "blue", "green", "orange", "purple"]

        for i, (rb_name, rb_data) in enumerate(self.data.items()):
            if len(rb_data["timestamps"]) > 0:
                x = np.array(rb_data["positions"]["x"])
                y = np.array(rb_data["positions"]["y"])
                z = np.array(rb_data["positions"]["z"])

                color = colors[i % len(colors)]
                ax.plot(x, y, z, label=f"{rb_name}", color=color, alpha=0.7)
                ax.scatter(
                    x[-1], y[-1], z[-1], color=color, s=100, marker="o"
                )  # 最后位置

        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Z (m)")
        ax.set_title("OptiTrack 3D Trajectories")
        ax.legend()
        plt.tight_layout()
        plt.show()

    def plot_time_series(self):
        """绘制时间序列图"""
        if not self.data:
            print("没有数据可绘制！")
            return

        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        fig.suptitle("OptiTrack Data", fontsize=16)

        colors = ["red", "blue", "green", "orange", "purple"]

        for i, (rb_name, rb_data) in enumerate(self.data.items()):
            if len(rb_data["timestamps"]) == 0:
                continue

            timestamps = np.array(rb_data["timestamps"])
            # 转换为相对时间（秒）
            timestamps = timestamps - timestamps[0]

            color = colors[i % len(colors)]

            # 位置数据
            axes[0, 0].plot(
                timestamps,
                rb_data["positions"]["x"],
                label=f"{rb_name} X",
                color=color,
                linestyle="-",
            )
            axes[0, 1].plot(
                timestamps,
                rb_data["positions"]["y"],
                label=f"{rb_name} Y",
                color=color,
                linestyle="-",
            )
            axes[1, 0].plot(
                timestamps,
                rb_data["positions"]["z"],
                label=f"{rb_name} Z",
                color=color,
                linestyle="-",
            )

            # 四元数模长（姿态稳定性指标）
            quat_norm = np.sqrt(
                np.array(rb_data["quaternions"]["w"]) ** 2
                + np.array(rb_data["quaternions"]["x"]) ** 2
                + np.array(rb_data["quaternions"]["y"]) ** 2
                + np.array(rb_data["quaternions"]["z"]) ** 2
            )
            axes[1, 1].plot(
                timestamps,
                quat_norm,
                label=f"{rb_name} Quat Norm",
                color=color,
                linestyle="-",
            )

        axes[0, 0].set_title("X")
        axes[0, 0].set_xlabel("t (s)")
        axes[0, 0].set_ylabel("X (m)")
        axes[0, 0].legend()
        axes[0, 0].grid(True)

        axes[0, 1].set_title("Y")
        axes[0, 1].set_xlabel("t (s)")
        axes[0, 1].set_ylabel("Y (m)")
        axes[0, 1].legend()
        axes[0, 1].grid(True)

        axes[1, 0].set_title("Z")
        axes[1, 0].set_xlabel("t (s)")
        axes[1, 0].set_ylabel("Z (m)")
        axes[1, 0].legend()
        axes[1, 0].grid(True)

        axes[1, 1].set_title("Quaternion Norm")
        axes[1, 1].set_xlabel("t (s)")
        axes[1, 1].set_ylabel("||Quaternion||")
        axes[1, 1].legend()
        axes[1, 1].grid(True)

        plt.tight_layout()
        plt.show()

    def print_statistics(self):
        """打印统计信息"""
        print("\n=== OptiTrack 数据统计 ===")
        for rb_name, rb_data in self.data.items():
            if len(rb_data["timestamps"]) == 0:
                continue

            print(f"\n刚体: {rb_name}")
            print(f"  数据点数量: {len(rb_data['timestamps'])}")

            # 位置统计
            x_pos = np.array(rb_data["positions"]["x"])
            y_pos = np.array(rb_data["positions"]["y"])
            z_pos = np.array(rb_data["positions"]["z"])

            print("  位置范围:")
            print(
                f"    X: {x_pos.min():.4f} ~ {x_pos.max():.4f} m (均值: {x_pos.mean():.4f})"
            )
            print(
                f"    Y: {y_pos.min():.4f} ~ {y_pos.max():.4f} m (均值: {y_pos.mean():.4f})"
            )
            print(
                f"    Z: {z_pos.min():.4f} ~ {z_pos.max():.4f} m (均值: {z_pos.mean():.4f})"
            )

            # 运动距离
            if len(x_pos) > 1:
                distances = np.sqrt(
                    np.diff(x_pos) ** 2 + np.diff(y_pos) ** 2 + np.diff(z_pos) ** 2
                )
                total_distance = np.sum(distances)
                print(f"  总运动距离: {total_distance:.4f} m")
                print(f"  平均速度: {np.mean(distances):.6f} m/sample")


def main():
    visualizer = OptiTrackVisualizer("../build/optitrack_target.txt")

    if not visualizer.parse_data():
        print("无法解析数据文件！")
        return

    # 打印统计信息
    visualizer.print_statistics()

    # 用户选择可视化类型
    print("\n请选择可视化类型:")
    print("1. 3D轨迹图")
    print("2. 时间序列图")
    print("3. 两者都显示")

    choice = input("请输入选择 (1/2/3): ").strip()

    if choice == "1":
        visualizer.plot_3d_trajectory()
    elif choice == "2":
        visualizer.plot_time_series()
    elif choice == "3":
        visualizer.plot_3d_trajectory()
        visualizer.plot_time_series()
    else:
        print("无效选择，显示所有图表")
        visualizer.plot_3d_trajectory()
        visualizer.plot_time_series()


if __name__ == "__main__":
    main()
