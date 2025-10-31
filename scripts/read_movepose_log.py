import re
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from datetime import datetime
from mpl_toolkits.mplot3d import Axes3D


def parse_movepose_log(filename):
    """解析MovePose的log输出"""
    move_data = []

    with open(filename, "r") as file:
        lines = file.readlines()

    i = 0
    while i < len(lines):
        line = lines[i].strip()

        # 查找MovePose开始行
        move_match = re.search(
            r"MovePose:\s+t\s*=\s*([\d.e-]+),\s+solve time\s*=\s*([\d.e-]+)", line
        )
        if move_match:
            t, solve_time = move_match.groups()
            t = float(t)
            solve_time = float(solve_time)

            # 查找关节角度行
            i += 1
            if i < len(lines):
                joint_line = lines[i].strip()
                joint_match = re.search(r"joint=\s*([-\d.e\s]+)", joint_line)
                if joint_match:
                    joint_values = [float(x) for x in joint_match.group(1).split()]

                    # 查找位置矩阵（4行）
                    pos_matrix = []
                    i += 2  # 跳过"pos ="行
                    for j in range(4):
                        if i + j < len(lines):
                            pos_line = lines[i + j].strip()
                            # 提取数值，忽略最后的标识符
                            pos_values = re.findall(r"[-\d.e]+", pos_line)
                            if len(pos_values) >= 4:
                                pos_matrix.append([float(x) for x in pos_values[:4]])

                    # 查找RPY行
                    i += 4
                    rpy = [0, 0, 0]  # 默认值
                    if i < len(lines):
                        rpy_line = lines[i].strip()
                        rpy_match = re.search(r"RPY=([-\d.e\s]+)", rpy_line)
                        if rpy_match:
                            rpy = [float(x) for x in rpy_match.group(1).split()]

                    # 提取位置和旋转信息
                    if len(pos_matrix) == 4:
                        position = (
                            pos_matrix[0][:3] + pos_matrix[1][:3] + pos_matrix[2][:3]
                        )  # 旋转矩阵前3列
                        translation = [
                            pos_matrix[0][3],
                            pos_matrix[1][3],
                            pos_matrix[2][3],
                        ]

                        move_data.append(
                            {
                                "time": t,
                                "solve_time": solve_time,
                                "joints": joint_values,
                                "position": translation,
                                "rotation_matrix": pos_matrix,
                                "rpy": rpy,
                            }
                        )
        i += 1

    return move_data


def analyze_movepose_data(data):
    """分析MovePose数据"""
    if not data:
        print("No MovePose data found")
        return

    df_list = []
    for entry in data:
        row = {
            "time": entry["time"],
            "solve_time": entry["solve_time"],
            "pos_x": entry["position"][0],
            "pos_y": entry["position"][1],
            "pos_z": entry["position"][2],
            "roll": entry["rpy"][0],
            "pitch": entry["rpy"][1],
            "yaw": entry["rpy"][2],
        }

        # 添加关节角度
        for i, joint in enumerate(entry["joints"]):
            row[f"joint_{i + 1}"] = joint

        df_list.append(row)

    df = pd.DataFrame(df_list)
    return df


def create_movepose_visualization(df):
    """创建MovePose数据可视化"""

    fig = plt.figure(figsize=(20, 15))

    # 1. 3D轨迹图
    ax1 = fig.add_subplot(3, 3, 1, projection="3d")
    ax1.plot(df["pos_x"], df["pos_y"], df["pos_z"], "b-", linewidth=2, alpha=0.7)
    ax1.scatter(
        df["pos_x"].iloc[0],
        df["pos_y"].iloc[0],
        df["pos_z"].iloc[0],
        c="green",
        s=100,
        label="Start",
    )
    ax1.scatter(
        df["pos_x"].iloc[-1],
        df["pos_y"].iloc[-1],
        df["pos_z"].iloc[-1],
        c="red",
        s=100,
        label="End",
    )
    ax1.set_xlabel("X Position")
    ax1.set_ylabel("Y Position")
    ax1.set_zlabel("Z Position")
    ax1.set_title("3D Trajectory")
    ax1.legend()

    # 2. 位置随时间变化
    ax2 = fig.add_subplot(3, 3, 2)
    ax2.plot(df["time"], df["pos_x"], "r-", label="X", linewidth=2)
    ax2.plot(df["time"], df["pos_y"], "g-", label="Y", linewidth=2)
    ax2.plot(df["time"], df["pos_z"], "b-", label="Z", linewidth=2)
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Position")
    ax2.set_title("Position vs Time")
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    # 3. RPY角度随时间变化
    ax3 = fig.add_subplot(3, 3, 3)
    ax3.plot(df["time"], df["roll"], "r-", label="Roll", linewidth=2)
    ax3.plot(df["time"], df["pitch"], "g-", label="Pitch", linewidth=2)
    ax3.plot(df["time"], df["yaw"], "b-", label="Yaw", linewidth=2)
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Angle (degrees)")
    ax3.set_title("RPY Angles vs Time")
    ax3.legend()
    ax3.grid(True, alpha=0.3)

    # 4. 关节角度变化（前3个关节）
    ax4 = fig.add_subplot(3, 3, 4)
    colors = ["red", "green", "blue"]
    for i in range(min(3, 6)):
        joint_col = f"joint_{i + 1}"
        if joint_col in df.columns:
            ax4.plot(
                df["time"],
                df[joint_col],
                color=colors[i],
                label=f"Joint {i + 1}",
                linewidth=2,
            )
    ax4.set_xlabel("Time (s)")
    ax4.set_ylabel("Joint Angle (degrees)")
    ax4.set_title("Joint Angles vs Time (1-3)")
    ax4.legend()
    ax4.grid(True, alpha=0.3)

    # 5. 关节角度变化（后3个关节）
    ax5 = fig.add_subplot(3, 3, 5)
    colors = ["orange", "purple", "brown"]
    for i in range(3, min(6, 6)):
        joint_col = f"joint_{i + 1}"
        if joint_col in df.columns:
            ax5.plot(
                df["time"],
                df[joint_col],
                color=colors[i - 3],
                label=f"Joint {i + 1}",
                linewidth=2,
            )
    ax5.set_xlabel("Time (s)")
    ax5.set_ylabel("Joint Angle (degrees)")
    ax5.set_title("Joint Angles vs Time (4-6)")
    ax5.legend()
    ax5.grid(True, alpha=0.3)

    # 6. 求解时间分析
    ax6 = fig.add_subplot(3, 3, 6)
    ax6.plot(df["time"], df["solve_time"] * 1000, "purple", linewidth=2)
    ax6.set_xlabel("Time (s)")
    ax6.set_ylabel("Solve Time (ms)")
    ax6.set_title("IK Solve Time vs Time")
    ax6.grid(True, alpha=0.3)

    # 7. XY平面轨迹
    ax7 = fig.add_subplot(3, 3, 7)
    scatter = ax7.scatter(
        df["pos_x"], df["pos_y"], c=df["time"], cmap="viridis", s=30, alpha=0.7
    )
    ax7.plot(df["pos_x"], df["pos_y"], "k-", alpha=0.3, linewidth=1)
    ax7.set_xlabel("X Position")
    ax7.set_ylabel("Y Position")
    ax7.set_title("XY Trajectory (colored by time)")
    plt.colorbar(scatter, ax=ax7, label="Time (s)")
    ax7.grid(True, alpha=0.3)

    # 8. 速度分析
    ax8 = fig.add_subplot(3, 3, 8)
    if len(df) > 1:
        # 计算位置变化率（近似速度）
        dt = np.diff(df["time"])
        dx = np.diff(df["pos_x"])
        dy = np.diff(df["pos_y"])
        dz = np.diff(df["pos_z"])

        velocity = np.sqrt(dx**2 + dy**2 + dz**2) / dt
        ax8.plot(df["time"][1:], velocity, "darkgreen", linewidth=2)
        ax8.set_xlabel("Time (s)")
        ax8.set_ylabel("Velocity Magnitude")
        ax8.set_title("Velocity vs Time")
        ax8.grid(True, alpha=0.3)

    # 9. 求解时间分布
    ax9 = fig.add_subplot(3, 3, 9)
    ax9.hist(
        df["solve_time"] * 1000, bins=20, alpha=0.7, color="orange", edgecolor="black"
    )
    ax9.set_xlabel("Solve Time (ms)")
    ax9.set_ylabel("Frequency")
    ax9.set_title("IK Solve Time Distribution")
    ax9.grid(True, alpha=0.3)

    plt.tight_layout()
    return fig


def print_movepose_stats(df):
    """打印MovePose统计信息"""
    print("\n=== MovePose Analysis Statistics ===")
    print("Total data points: {len(df)}")
    print("Time duration: {df['time'].max() - df['time'].min():.3f} seconds")
    print("Average time step: {df['time'].diff().mean():.6f} seconds")

    print("\n=== Position Statistics ===")
    print("X range: {df['pos_x'].min():.6f} ~ {df['pos_x'].max():.6f}")
    print("Y range: {df['pos_y'].min():.6f} ~ {df['pos_y'].max():.6f}")
    print("Z range: {df['pos_z'].min():.6f} ~ {df['pos_z'].max():.6f}")

    # 计算总运动距离
    if len(df) > 1:
        distances = np.sqrt(
            np.diff(df["pos_x"]) ** 2
            + np.diff(df["pos_y"]) ** 2
            + np.diff(df["pos_z"]) ** 2
        )
        total_distance = np.sum(distances)
        print(f"Total movement distance: {total_distance:.6f}")

    print("\n=== RPY Statistics (degrees) ===")
    print(f"Roll range: {df['roll'].min():.2f}° ~ {df['roll'].max():.2f}°")
    print(f"Pitch range: {df['pitch'].min():.2f}° ~ {df['pitch'].max():.2f}°")
    print(f"Yaw range: {df['yaw'].min():.2f}° ~ {df['yaw'].max():.2f}°")

    print("\n=== Joint Angle Statistics (degrees) ===")
    for i in range(6):
        joint_col = f"joint_{i + 1}"
        if joint_col in df.columns:
            print(
                f"Joint {i + 1}: {df[joint_col].min():.2f}° ~ {df[joint_col].max():.2f}°"
            )

    print("\n=== IK Solve Time Statistics ===")
    solve_times_ms = df["solve_time"] * 1000
    print(f"Average solve time: {solve_times_ms.mean():.3f} ms")
    print(f"Max solve time: {solve_times_ms.max():.3f} ms")
    print(f"Min solve time: {solve_times_ms.min():.3f} ms")
    print(f"Std deviation: {solve_times_ms.std():.3f} ms")

    # 检测异常求解时间
    threshold = solve_times_ms.mean() + 2 * solve_times_ms.std()
    outliers = df[solve_times_ms > threshold]
    if len(outliers) > 0:
        print(f"\nSlow solve instances (>{threshold:.3f}ms): {len(outliers)}")
        for idx, row in outliers.iterrows():
            print(f"  t={row['time']:.3f}s: {row['solve_time'] * 1000:.3f}ms")


def main():
    """主函数"""
    # 从标准输入或文件读取log
    import sys

    if len(sys.argv) > 1:
        filename = sys.argv[1]
        print(f"Reading from file: {filename}")
        try:
            data = parse_movepose_log(filename)
        except FileNotFoundError:
            print(f"File {filename} not found")
            return
    else:
        filename = "../build/movepose_log.txt"
        data = parse_movepose_log(filename)

    if not data:
        print("No MovePose data found in the log file")
        return

    print(f"Parsed {len(data)} MovePose entries")

    # 分析数据
    df = analyze_movepose_data(data)

    # 打印统计信息
    print_movepose_stats(df)

    # 创建可视化
    print("\nCreating visualization...")
    fig = create_movepose_visualization(df)

    # 保存图片和数据
    fig.savefig("movepose_analysis.png", dpi=300, bbox_inches="tight")
    df.to_csv("movepose_data.csv", index=False)

    print("\nOutput files:")
    print("- movepose_analysis.png: Visualization charts")
    print("- movepose_data.csv: Processed data")

    # 显示图表
    plt.show()


if __name__ == "__main__":
    main()
