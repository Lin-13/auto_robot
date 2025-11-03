import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import re
import os
from collections import deque
import sys
# 在debug打印的Eigen会有[]


class RealTimeDataPlotter:
    def __init__(self, filename, max_points=1000):
        """
        实时数据绘图器
        Args:
            filename: 日志文件路径
            max_points: 最大显示数据点数量
        """
        self.filename = filename
        self.max_points = max_points
        self.last_position = 0  # 记录上次读取的文件位置

        # 使用deque存储数据，限制最大长度
        self.force_data = deque(maxlen=max_points)
        self.xyz_plan_data = deque(maxlen=max_points)
        self.joint_plan_data = deque(maxlen=max_points)
        self.actual_xyz_data = deque(maxlen=max_points)
        self.actual_joint_data = deque(maxlen=max_points)

        # 创建图形和子图
        self.fig, self.axes = plt.subplots(2, 2, figsize=(16, 12))
        self.fig.suptitle("Real-time Robot Motion Data", fontsize=16)

        # 初始化图表
        self.setup_plots()

    def setup_plots(self):
        """设置图表"""
        # 力的时间变化图 (左上)
        self.force_lines = []
        colors = ["r", "g", "b"]
        labels = ["Fx", "Fy", "Fz"]
        for i, (color, label) in enumerate(zip(colors, labels)):
            (line,) = self.axes[0, 0].plot(
                [], [], color=color, label=label, linewidth=2
            )
            self.force_lines.append(line)
        self.axes[0, 0].set_xlabel("Time (s)")
        self.axes[0, 0].set_ylabel("Force (N)")
        self.axes[0, 0].set_title("Force")
        self.axes[0, 0].legend()
        self.axes[0, 0].grid(True, alpha=0.3)

        # 位置对比 (右上)
        self.position_lines = []
        colors = ["r", "g", "b", "r", "g", "b"]
        labels = ["Plan X", "Plan Y", "Plan Z", "Actual X", "Actual Y", "Actual Z"]
        styles = ["--", "--", "--", "-", "-", "-"]
        for i, (color, label, style) in enumerate(zip(colors, labels, styles)):
            (line,) = self.axes[0, 1].plot(
                [],
                [],
                color=color,
                linestyle=style,
                label=label,
                linewidth=2,
                alpha=0.7 if style == "--" else 1.0,
            )
            self.position_lines.append(line)
        self.axes[0, 1].set_xlabel("Time (s)")
        self.axes[0, 1].set_ylabel("Position (mm)")
        self.axes[0, 1].set_title("Position (Plan vs Actual)")
        self.axes[0, 1].legend()
        self.axes[0, 1].grid(True, alpha=0.3)

        # 计划关节角度 (左下)
        self.joint_plan_lines = []
        joint_colors = ["red", "green", "blue", "orange", "purple", "brown"]
        for i in range(6):
            (line,) = self.axes[1, 0].plot(
                [], [], color=joint_colors[i], label=f"Joint {i + 1}", linewidth=2
            )
            self.joint_plan_lines.append(line)
        self.axes[1, 0].set_xlabel("Time (s)")
        self.axes[1, 0].set_ylabel("Joint Angle (degrees)")
        self.axes[1, 0].set_title("Planned Joint Angles")
        self.axes[1, 0].legend()
        self.axes[1, 0].grid(True, alpha=0.3)

        # 实际关节角度 (右下)
        self.joint_actual_lines = []
        for i in range(6):
            (line,) = self.axes[1, 1].plot(
                [], [], color=joint_colors[i], label=f"Joint {i + 1}", linewidth=2
            )
            self.joint_actual_lines.append(line)
        self.axes[1, 1].set_xlabel("Time (s)")
        self.axes[1, 1].set_ylabel("Joint Angle (degrees)")
        self.axes[1, 1].set_title("Actual Joint Angles")
        self.axes[1, 1].legend()
        self.axes[1, 1].grid(True, alpha=0.3)

    def parse_new_data(self):
        """解析新的数据"""
        if not os.path.exists(self.filename):
            return

        try:
            with open(self.filename, "r") as file:
                # 移动到上次读取的位置
                file.seek(self.last_position)
                new_lines = file.readlines()
                # 更新位置
                self.last_position = file.tell()

            for line in new_lines:
                line = line.strip()
                if not line:
                    continue

                try:
                    # 解析力数据
                    force_match = re.search(
                        r"t\s*:\s*([\d.]+)\s+force\s*:\s*\[([-\d.e]+),([-\d.e]+),([-\d.e]+)\]",
                        line,
                    )
                    if force_match:
                        t, fx, fy, fz = force_match.groups()
                        self.force_data.append(
                            [float(t), float(fx), float(fy), float(fz)]
                        )

                    # 解析xyz(plan)数据
                    xyz_match = re.search(
                        r"t\s*:\s*([\d.]+)\s+xyz\s*:\s*\[([-\d.e]+),([-\d.e]+),([-\d.e]+)\]",
                        line,
                    )
                    if xyz_match:
                        t, x, y, z = xyz_match.groups()
                        self.xyz_plan_data.append(
                            [float(t), float(x), float(y), float(z)]
                        )

                    # 解析关节数据
                    joint_match = re.search(
                        r"t\s*:\s*([\d.]+)\s+joint\s*:\s*\[([-\d.e]+),([-\d.e]+),([-\d.e]+),([-\d.e]+),([-\d.e]+),([-\d.e]+)\]",
                        line,
                    )
                    if joint_match:
                        t, j1, j2, j3, j4, j5, j6 = joint_match.groups()
                        self.joint_plan_data.append(
                            [
                                float(t),
                                float(j1),
                                float(j2),
                                float(j3),
                                float(j4),
                                float(j5),
                                float(j6),
                            ]
                        )

                    # 解析actual_xyz数据
                    actual_xyz_match = re.search(
                        r"t\s*:\s*([\d.]+)\s+actual_xyz\s*:\s*\[([-\d.e]+),([-\d.e]+),([-\d.e]+)\]",
                        line,
                    )
                    if actual_xyz_match:
                        t, x, y, z = actual_xyz_match.groups()
                        self.actual_xyz_data.append(
                            [float(t), float(x), float(y), float(z)]
                        )

                    # 解析actual_joint数据
                    actual_joint_match = re.search(
                        r"t\s*:\s*([\d.]+)\s+actual_joint\s*:\s*\[([-\d.e]+),([-\d.e]+),([-\d.e]+),([-\d.e]+),([-\d.e]+),([-\d.e]+)\]",
                        line,
                    )
                    if actual_joint_match:
                        t, j1, j2, j3, j4, j5, j6 = actual_joint_match.groups()
                        self.actual_joint_data.append(
                            [
                                float(t),
                                float(j1),
                                float(j2),
                                float(j3),
                                float(j4),
                                float(j5),
                                float(j6),
                            ]
                        )

                except Exception as e:
                    print(f"解析行时出错: {line[:50]}... 错误: {e}")
                    continue

        except Exception as e:
            print(f"读取文件时出错: {e}")

    def update_plots(self, frame):
        """更新图表"""
        # 解析新数据
        self.parse_new_data()

        # 更新力数据图
        if self.force_data:
            force_array = np.array(self.force_data)
            times = force_array[:, 0]
            for i, line in enumerate(self.force_lines):
                line.set_data(times, force_array[:, i + 1])

            # 自动调整坐标轴
            self.axes[0, 0].relim()
            self.axes[0, 0].autoscale_view()

        # 更新位置数据图
        position_data_ready = (
            len(self.xyz_plan_data) > 0 or len(self.actual_xyz_data) > 0
        )
        if position_data_ready:
            # 计划位置数据
            if self.xyz_plan_data:
                xyz_plan_array = np.array(self.xyz_plan_data)
                times_plan = xyz_plan_array[:, 0]
                for i in range(3):
                    self.position_lines[i].set_data(
                        times_plan, xyz_plan_array[:, i + 1] * 1000
                    )

            # 实际位置数据
            if self.actual_xyz_data:
                actual_xyz_array = np.array(self.actual_xyz_data)
                times_actual = actual_xyz_array[:, 0]
                for i in range(3):
                    self.position_lines[i + 3].set_data(
                        times_actual, actual_xyz_array[:, i + 1] * 1000
                    )

            self.axes[0, 1].relim()
            self.axes[0, 1].autoscale_view()

        # 更新计划关节数据图
        if self.joint_plan_data:
            joint_plan_array = np.array(self.joint_plan_data)
            times = joint_plan_array[:, 0]
            for i, line in enumerate(self.joint_plan_lines):
                if i + 1 < joint_plan_array.shape[1]:
                    line.set_data(times, np.degrees(joint_plan_array[:, i + 1]))

            self.axes[1, 0].relim()
            self.axes[1, 0].autoscale_view()

        # 更新实际关节数据图
        if self.actual_joint_data:
            actual_joint_array = np.array(self.actual_joint_data)
            times = actual_joint_array[:, 0]
            for i, line in enumerate(self.joint_actual_lines):
                if i + 1 < actual_joint_array.shape[1]:
                    line.set_data(times, np.degrees(actual_joint_array[:, i + 1]))

            self.axes[1, 1].relim()
            self.axes[1, 1].autoscale_view()

        # 显示当前数据统计
        total_points = (
            len(self.force_data)
            + len(self.xyz_plan_data)
            + len(self.joint_plan_data)
            + len(self.actual_xyz_data)
            + len(self.actual_joint_data)
        )
        self.fig.suptitle(
            f"Real-time Robot Motion Data (Total Points: {total_points})", fontsize=16
        )

        return (
            self.force_lines
            + self.position_lines
            + self.joint_plan_lines
            + self.joint_actual_lines
        )

    def start_animation(self):
        """开始动画"""
        # 每1000ms(1秒)更新一次
        self.ani = animation.FuncAnimation(
            self.fig,
            self.update_plots,
            interval=1000,
            blit=False,
            cache_frame_data=False,
        )
        return self.ani


def parse_pose_log_simple(filename):
    """保留原有的静态解析函数用于兼容"""
    force_data = []
    xyz_data = []
    joint_data = []
    actual_xyz_data = []
    actual_joint_data = []

    with open(filename, "r") as file:
        lines = file.readlines()

    for line in lines:
        line = line.strip()
        if not line:
            continue

        try:
            # 解析力数据: t : 0 force : [0.0380876,-0.0368688,0.0187793]
            force_match = re.search(
                r"t\s*:\s*([\d.]+)\s+force\s*:\s*\[([-\d.e]+),([-\d.e]+),([-\d.e]+)\]",
                line,
            )
            if force_match:
                t, fx, fy, fz = force_match.groups()
                force_data.append([float(t), float(fx), float(fy), float(fz)])

            # 解析xyz(plan)数据: t : 0 xyz : [1.54149e-07,-2.26314e-08,-1.97827e-08]
            xyz_match = re.search(
                r"t\s*:\s*([\d.]+)\s+xyz\s*:\s*\[([-\d.e]+),([-\d.e]+),([-\d.e]+)\]",
                line,
            )
            if xyz_match:
                t, x, y, z = xyz_match.groups()
                xyz_data.append([float(t), float(x), float(y), float(z)])

            # 解析关节数据: t : 0 joint : [1.10701,0.437816,-1.20056,1.493,-0.584533,-3.96152]
            joint_match = re.search(
                r"t\s*:\s*([\d.]+)\s+joint\s*:\s*\[([-\d.e]+),([-\d.e]+),([-\d.e]+),([-\d.e]+),([-\d.e]+),([-\d.e]+)\]",
                line,
            )
            if joint_match:
                t, j1, j2, j3, j4, j5, j6 = joint_match.groups()
                joint_data.append(
                    [
                        float(t),
                        float(j1),
                        float(j2),
                        float(j3),
                        float(j4),
                        float(j5),
                        float(j6),
                    ]
                )

            # 解析actual_xyz数据: t : 0 actual_xyz : [0,0,0]
            actual_xyz_match = re.search(
                r"t\s*:\s*([\d.]+)\s+actual_xyz\s*:\s*\[([-\d.e]+),([-\d.e]+),([-\d.e]+)\]",
                line,
            )
            if actual_xyz_match:
                t, x, y, z = actual_xyz_match.groups()
                actual_xyz_data.append([float(t), float(x), float(y), float(z)])

            # 解析actual_joint数据: t : 0 actual_joint : [1.10701,0.437816,-1.20056,1.493,-0.584533,-3.96152]
            actual_joint_match = re.search(
                r"t\s*:\s*([\d.]+)\s+actual_joint\s*:\s*\[([-\d.e]+),([-\d.e]+),([-\d.e]+),([-\d.e]+),([-\d.e]+),([-\d.e]+)\]",
                line,
            )
            if actual_joint_match:
                t, j1, j2, j3, j4, j5, j6 = actual_joint_match.groups()
                actual_joint_data.append(
                    [
                        float(t),
                        float(j1),
                        float(j2),
                        float(j3),
                        float(j4),
                        float(j5),
                        float(j6),
                    ]
                )

        except Exception as e:
            print(f"解析行时出错: {line[:50]}... 错误: {e}")
            continue

    return {
        "force": np.array(force_data) if force_data else np.array([]),
        "xyz_plan": np.array(xyz_data) if xyz_data else np.array([]),
        "joint_plan": np.array(joint_data) if joint_data else np.array([]),
        "actual_xyz": np.array(actual_xyz_data) if actual_xyz_data else np.array([]),
        "actual_joint": np.array(actual_joint_data)
        if actual_joint_data
        else np.array([]),
    }


def create_visualization(data):
    """创建可视化图表"""

    # 检查是否有数据
    if (
        len(data["force"]) == 0
        and len(data["xyz_plan"]) == 0
        and len(data["joint_plan"]) == 0
        and len(data["actual_xyz"]) == 0
    ):
        print("No valid data found for visualization")
        return

    # 创建子图 - 4个子图，关节角度使用2个子图
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    fig.suptitle("Robot Motion Data Visualization", fontsize=16)

    # 1. 力的时间变化图 (左上)
    if len(data["force"]) > 0:
        axes[0, 0].plot(
            data["force"][:, 0], data["force"][:, 1], "r-", label="Fx", linewidth=2
        )
        axes[0, 0].plot(
            data["force"][:, 0], data["force"][:, 2], "g-", label="Fy", linewidth=2
        )
        axes[0, 0].plot(
            data["force"][:, 0], data["force"][:, 3], "b-", label="Fz", linewidth=2
        )
        axes[0, 0].set_xlabel("Time (s)")
        axes[0, 0].set_ylabel("Force (N)")
        axes[0, 0].set_title("Force")
        axes[0, 0].legend()
        axes[0, 0].grid(True, alpha=0.3)
    else:
        axes[0, 0].text(
            0.5,
            0.5,
            "No Force Data",
            ha="center",
            va="center",
            transform=axes[0, 0].transAxes,
        )

    # 2. 位置对比 (右上)
    if len(data["xyz_plan"]) > 0 and len(data["actual_xyz"]) > 0:
        # 计划位置
        axes[0, 1].plot(
            data["xyz_plan"][:, 0],
            data["xyz_plan"][:, 1] * 1000,
            "r--",
            label="Plan X",
            linewidth=2,
            alpha=0.7,
        )
        axes[0, 1].plot(
            data["xyz_plan"][:, 0],
            data["xyz_plan"][:, 2] * 1000,
            "g--",
            label="Plan Y",
            linewidth=2,
            alpha=0.7,
        )
        axes[0, 1].plot(
            data["xyz_plan"][:, 0],
            data["xyz_plan"][:, 3] * 1000,
            "b--",
            label="Plan Z",
            linewidth=2,
            alpha=0.7,
        )
        # 实际位置
        axes[0, 1].plot(
            data["actual_xyz"][:, 0],
            data["actual_xyz"][:, 1] * 1000,
            "r-",
            label="Actual X",
            linewidth=2,
        )
        axes[0, 1].plot(
            data["actual_xyz"][:, 0],
            data["actual_xyz"][:, 2] * 1000,
            "g-",
            label="Actual Y",
            linewidth=2,
        )
        axes[0, 1].plot(
            data["actual_xyz"][:, 0],
            data["actual_xyz"][:, 3] * 1000,
            "b-",
            label="Actual Z",
            linewidth=2,
        )
        axes[0, 1].set_xlabel("Time (s)")
        axes[0, 1].set_ylabel("Position (mm)")
        axes[0, 1].set_title("Position (Plan vs Actual)")
        axes[0, 1].legend()
        axes[0, 1].grid(True, alpha=0.3)
    else:
        axes[0, 1].text(
            0.5,
            0.5,
            "No Position Data",
            ha="center",
            va="center",
            transform=axes[0, 1].transAxes,
        )

    # 3. 计划关节角度 (左下)
    if len(data["joint_plan"]) > 0:
        joint_colors = ["red", "green", "blue", "orange", "purple", "brown"]
        for i in range(min(6, data["joint_plan"].shape[1] - 1)):  # -1因为第一列是时间
            axes[1, 0].plot(
                data["joint_plan"][:, 0],
                np.degrees(data["joint_plan"][:, i + 1]),
                color=joint_colors[i],
                label=f"Joint {i + 1}",
                linewidth=2,
            )
        axes[1, 0].set_xlabel("Time (s)")
        axes[1, 0].set_ylabel("Joint Angle (degrees)")
        axes[1, 0].set_title("Planned Joint Angles")
        axes[1, 0].legend()
        axes[1, 0].grid(True, alpha=0.3)
    else:
        axes[1, 0].text(
            0.5,
            0.5,
            "No Planned Joint Data",
            ha="center",
            va="center",
            transform=axes[1, 0].transAxes,
        )

    # 4. 实际关节角度 (右下)
    if len(data["actual_joint"]) > 0:
        joint_colors = ["red", "green", "blue", "orange", "purple", "brown"]
        for i in range(min(6, data["actual_joint"].shape[1] - 1)):  # -1因为第一列是时间
            axes[1, 1].plot(
                data["actual_joint"][:, 0],
                np.degrees(data["actual_joint"][:, i + 1]),
                color=joint_colors[i],
                label=f"Joint {i + 1}",
                linewidth=2,
            )
        axes[1, 1].set_xlabel("Time (s)")
        axes[1, 1].set_ylabel("Joint Angle (degrees)")
        axes[1, 1].set_title("Actual Joint Angles")
        axes[1, 1].legend()
        axes[1, 1].grid(True, alpha=0.3)
    else:
        axes[1, 1].text(
            0.5,
            0.5,
            "No Actual Joint Data",
            ha="center",
            va="center",
            transform=axes[1, 1].transAxes,
        )

    plt.tight_layout()
    return fig


def print_basic_stats(data):
    """打印基本统计信息"""
    print("\n=== Basic Statistics ===")

    for data_type, array in data.items():
        if len(array) > 0:
            time_span = array[-1, 0] - array[0, 0] if len(array) > 1 else 0
            print(f"{data_type}: {len(array)} data points, duration: {time_span:.1f}s")

            if data_type == "force":
                print(
                    f"  Fx range: {array[:, 1].min():.3f} ~ {array[:, 1].max():.3f} N"
                )
                print(
                    f"  Fy range: {array[:, 2].min():.3f} ~ {array[:, 2].max():.3f} N"
                )
                print(
                    f"  Fz range: {array[:, 3].min():.3f} ~ {array[:, 3].max():.3f} N"
                )

            elif "xyz" in data_type:
                print(
                    f"  X range: {array[:, 1].min() * 1000:.3f} ~ {array[:, 1].max() * 1000:.3f} mm"
                )
                print(
                    f"  Y range: {array[:, 2].min() * 1000:.3f} ~ {array[:, 2].max() * 1000:.3f} mm"
                )
                print(
                    f"  Z range: {array[:, 3].min() * 1000:.3f} ~ {array[:, 3].max() * 1000:.3f} mm"
                )

            elif "joint" in data_type:
                print("  Joint angle ranges (degrees):")
                for i in range(1, min(7, array.shape[1])):
                    degrees = np.degrees(array[:, i])
                    print(f"    Joint {i}: {degrees.min():.1f}° ~ {degrees.max():.1f}°")
        else:
            print(f"{data_type}: No data")


def main():
    """主函数"""
    filename = "../build/pose_log.txt"

    if len(sys.argv) > 1:
        try:
            choice = sys.argv[1]
        except (IndexError, ValueError):
            print("用法: python read_pose_debug.py [1|2]")
            print("  1 - 实时模式")
            print("  2 - 静态模式")
            return
    else:
        # 如果没有提供参数，使用交互式选择
        print("选择模式:")
        print("1. 实时模式 - 每秒读取并更新图像")
        print("2. 静态模式 - 一次性读取所有数据")
        choice = input("请输入选择 (1 或 2): ").strip()

    try:
        if choice == "1":
            print("启动实时模式...")
            print("程序将每秒读取新数据并更新图像")
            print("关闭图表窗口可停止程序")

            # 创建实时绘图器
            plotter = RealTimeDataPlotter(filename, max_points=1000)

            # 开始动画
            ani = plotter.start_animation()

            # 显示图表
            plt.tight_layout()
            plt.show()

        elif choice == "2":
            print("启动静态模式...")

            data = parse_pose_log_simple(filename)

            # 检查是否解析到数据
            total_points = sum(len(arr) for arr in data.values())
            if total_points == 0:
                print("No valid data found, please check log file format")
                return

            print("Successfully parsed data!")

            # 打印基本统计信息
            print_basic_stats(data)

            # 创建可视化
            print("\nCreating visualization...")
            fig = create_visualization(data)

            if fig:
                # 保存图片
                fig.savefig("robot_motion_analysis.png", dpi=300, bbox_inches="tight")
                print("Chart saved as: robot_motion_analysis.png")

                # 显示图表
                plt.show()
        else:
            print("无效选择，请输入 1 或 2")

    except KeyboardInterrupt:
        print("\n程序被用户中断")
    except Exception as e:
        print(f"Error: {e}")
        import traceback

        traceback.print_exc()


if __name__ == "__main__":
    main()
