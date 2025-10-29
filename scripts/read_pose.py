import matplotlib.pyplot as plt
import numpy as np
import re


def parse_pose_log_simple(filename):
    """解析pose log数据"""
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
            # 解析力数据: t : 0 force :  0.114755 -0.155582 0.0242094
            force_match = re.search(
                r"t\s*:\s*([\d.]+)\s+force\s*:\s*([-\d.e]+)\s+([-\d.e]+)\s+([-\d.e]+)",
                line,
            )
            if force_match:
                t, fx, fy, fz = force_match.groups()
                force_data.append([float(t), float(fx), float(fy), float(fz)])

            # 解析xyz(plan)数据: t : 0 xyz :    4.878e-09 -2.36767e-08  8.95495e-09
            xyz_match = re.search(
                r"t\s*:\s*([\d.]+)\s+xyz\s*:\s*([-\d.e]+)\s+([-\d.e]+)\s+([-\d.e]+)",
                line,
            )
            if xyz_match:
                t, x, y, z = xyz_match.groups()
                xyz_data.append([float(t), float(x), float(y), float(z)])

            # 解析关节数据: t : 0 joint :  1.44606 0.396738 -1.09864  1.10692  1.03131 -3.66985
            joint_match = re.search(
                r"t\s*:\s*([\d.]+)\s+joint\s*:\s*([-\d.e]+)\s+([-\d.e]+)\s+([-\d.e]+)\s+([-\d.e]+)\s+([-\d.e]+)\s+([-\d.e]+)",
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

            # 解析actual_xyz数据: t : 0 actual_xyz :  0.156304 -0.715972  0.578374
            actual_xyz_match = re.search(
                r"t\s*:\s*([\d.]+)\s+actual_xyz\s*:\s*([-\d.e]+)\s+([-\d.e]+)\s+([-\d.e]+)",
                line,
            )
            if actual_xyz_match:
                t, x, y, z = actual_xyz_match.groups()
                actual_xyz_data.append([float(t), float(x), float(y), float(z)])

            # 解析actual_joint数据: t : 0 actual_joint :  1.44606 0.396738 -1.09864  1.10692  1.03131 -3.66985
            actual_joint_match = re.search(
                r"t\s*:\s*([\d.]+)\s+actual_joint\s*:\s*([-\d.e]+)\s+([-\d.e]+)\s+([-\d.e]+)\s+([-\d.e]+)\s+([-\d.e]+)\s+([-\d.e]+)",
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
                print(f"  Joint angle ranges (degrees):")
                for i in range(1, min(7, array.shape[1])):
                    degrees = np.degrees(array[:, i])
                    print(f"    Joint {i}: {degrees.min():.1f}° ~ {degrees.max():.1f}°")
        else:
            print(f"{data_type}: No data")


def main():
    """主函数"""
    filename = "../build/pose_log.txt"

    print("Parsing data...")
    try:
        data = parse_pose_log_simple(filename)

        # 检查是否解析到数据
        total_points = sum(len(arr) for arr in data.values())
        if total_points == 0:
            print("No valid data found, please check log file format")
            return

        print(f"Successfully parsed data!")

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

    except Exception as e:
        print(f"Error: {e}")
        import traceback

        traceback.print_exc()


if __name__ == "__main__":
    main()
