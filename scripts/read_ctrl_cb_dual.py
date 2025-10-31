import re
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rcParams
import warnings

warnings.filterwarnings("ignore")  # 忽略matplotlib无关警告

# 配置中文字体（避免中文乱码）
rcParams["font.sans-serif"] = ["DejaVu Sans", "SimHei", "Arial Unicode MS"]
rcParams["axes.unicode_minus"] = False  # 解决负号显示异常


def extract_joint_and_time_dual_robot(log_file_path):
    """
    从日志文件提取双机器人关节状态向量与对应的timer_cb时间，返回关联数据

    参数:
        log_file_path (str): 日志文件路径

    返回:
        dict: 包含左右机器人数据的字典
    """
    # 正则表达式：匹配timer_cb时间（区分left和right）
    left_timer_pattern = re.compile(
        r"aubo left_aubo: timer_cb at\s+([-+]?\d+\.?\d*e?[-+]?\d*)\s+s"
    )
    right_timer_pattern = re.compile(
        r"aubo right_aubo: timer_cb at\s+([-+]?\d+\.?\d*e?[-+]?\d*)\s+s"
    )

    # 正则表达式：匹配关节状态（区分left和right）
    left_get_joint_pattern = re.compile(
        r"aubo left_aubo: get joint state\s+([-+]?\d+\.\d+)\s+([-+]?\d+\.?\d*e?[-+]?\d*)\s+([-+]?\d+\.\d+)\s+([-+]?\d+\.\d+)\s+([-+]?\d+\.\d+)\s+([-+]?\d+\.?\d*e?[-+]?\d*)"
    )
    right_get_joint_pattern = re.compile(
        r"aubo right_aubo: get joint state\s+([-+]?\d+\.\d+)\s+([-+]?\d+\.?\d*e?[-+]?\d*)\s+([-+]?\d+\.\d+)\s+([-+]?\d+\.\d+)\s+([-+]?\d+\.\d+)\s+([-+]?\d+\.?\d*e?[-+]?\d*)"
    )

    left_set_joint_pattern = re.compile(
        r"aubo left_aubo: set joint state\s+([-+]?\d+\.\d+)\s+([-+]?\d+\.?\d*e?[-+]?\d*)\s+([-+]?\d+\.\d+)\s+([-+]?\d+\.\d+)\s+([-+]?\d+\.\d+)\s+([-+]?\d+\.?\d*e?[-+]?\d*)"
    )
    right_set_joint_pattern = re.compile(
        r"aubo right_aubo: set joint state\s+([-+]?\d+\.\d+)\s+([-+]?\d+\.?\d*e?[-+]?\d*)\s+([-+]?\d+\.\d+)\s+([-+]?\d+\.\d+)\s+([-+]?\d+\.\d+)\s+([-+]?\d+\.?\d*e?[-+]?\d*)"
    )

    left_set_target_pattern = re.compile(
        r"aubo left_aubo: set target\s+([-+]?\d+\.\d+)\s+([-+]?\d+\.?\d*e?[-+]?\d*)\s+([-+]?\d+\.\d+)\s+([-+]?\d+\.\d+)\s+([-+]?\d+\.\d+)\s+([-+]?\d+\.?\d*e?[-+]?\d*)"
    )
    right_set_target_pattern = re.compile(
        r"aubo right_aubo: set target\s+([-+]?\d+\.\d+)\s+([-+]?\d+\.?\d*e?[-+]?\d*)\s+([-+]?\d+\.\d+)\s+([-+]?\d+\.\d+)\s+([-+]?\d+\.\d+)\s+([-+]?\d+\.?\d*e?[-+]?\d*)"
    )

    # 数据存储
    left_data = {"get": [], "set": [], "target": []}
    right_data = {"get": [], "set": [], "target": []}

    invalid_lines = []
    left_last_timer_time = None
    right_last_timer_time = None

    try:
        with open(log_file_path, "r", encoding="utf-8") as f:
            for line_num, line in enumerate(f, 1):
                line_stripped = line.strip()

                # 1. 提取left机器人timer_cb时间
                left_time_match = left_timer_pattern.search(line_stripped)
                if left_time_match:
                    try:
                        left_last_timer_time = float(left_time_match.group(1))
                    except ValueError:
                        invalid_lines.append(
                            f"第{line_num}行：left timer_cb时间转换失败 - {line_stripped}"
                        )
                    continue

                # 2. 提取right机器人timer_cb时间
                right_time_match = right_timer_pattern.search(line_stripped)
                if right_time_match:
                    try:
                        right_last_timer_time = float(right_time_match.group(1))
                    except ValueError:
                        invalid_lines.append(
                            f"第{line_num}行：right timer_cb时间转换失败 - {line_stripped}"
                        )
                    continue

                # 3. 提取left机器人关节状态
                left_get_match = left_get_joint_pattern.search(line_stripped)
                if left_get_match:
                    if left_last_timer_time is None:
                        invalid_lines.append(
                            f"第{line_num}行：left get关节状态无对应timer_cb时间 - {line_stripped}"
                        )
                        continue
                    try:
                        joints = [float(left_get_match.group(i)) for i in range(1, 7)]
                        left_data["get"].append([left_last_timer_time] + joints)
                    except ValueError:
                        invalid_lines.append(
                            f"第{line_num}行：left get关节角度转换失败 - {line_stripped}"
                        )

                left_set_match = left_set_joint_pattern.search(line_stripped)
                if left_set_match:
                    if left_last_timer_time is None:
                        invalid_lines.append(
                            f"第{line_num}行：left set关节状态无对应timer_cb时间 - {line_stripped}"
                        )
                        continue
                    try:
                        joints = [float(left_set_match.group(i)) for i in range(1, 7)]
                        left_data["set"].append([left_last_timer_time] + joints)
                    except ValueError:
                        invalid_lines.append(
                            f"第{line_num}行：left set关节角度转换失败 - {line_stripped}"
                        )

                left_target_match = left_set_target_pattern.search(line_stripped)
                if left_target_match:
                    if left_last_timer_time is None:
                        invalid_lines.append(
                            f"第{line_num}行：left set target关节状态无对应timer_cb时间 - {line_stripped}"
                        )
                        continue
                    try:
                        joints = [
                            float(left_target_match.group(i)) for i in range(1, 7)
                        ]
                        left_data["target"].append([left_last_timer_time] + joints)
                    except ValueError:
                        invalid_lines.append(
                            f"第{line_num}行：left set target关节角度转换失败 - {line_stripped}"
                        )

                # 4. 提取right机器人关节状态
                right_get_match = right_get_joint_pattern.search(line_stripped)
                if right_get_match:
                    if right_last_timer_time is None:
                        invalid_lines.append(
                            f"第{line_num}行：right get关节状态无对应timer_cb时间 - {line_stripped}"
                        )
                        continue
                    try:
                        joints = [float(right_get_match.group(i)) for i in range(1, 7)]
                        right_data["get"].append([right_last_timer_time] + joints)
                    except ValueError:
                        invalid_lines.append(
                            f"第{line_num}行：right get关节角度转换失败 - {line_stripped}"
                        )

                right_set_match = right_set_joint_pattern.search(line_stripped)
                if right_set_match:
                    if right_last_timer_time is None:
                        invalid_lines.append(
                            f"第{line_num}行：right set关节状态无对应timer_cb时间 - {line_stripped}"
                        )
                        continue
                    try:
                        joints = [float(right_set_match.group(i)) for i in range(1, 7)]
                        right_data["set"].append([right_last_timer_time] + joints)
                    except ValueError:
                        invalid_lines.append(
                            f"第{line_num}行：right set关节角度转换失败 - {line_stripped}"
                        )

                right_target_match = right_set_target_pattern.search(line_stripped)
                if right_target_match:
                    if right_last_timer_time is None:
                        invalid_lines.append(
                            f"第{line_num}行：right set target关节状态无对应timer_cb时间 - {line_stripped}"
                        )
                        continue
                    try:
                        joints = [
                            float(right_target_match.group(i)) for i in range(1, 7)
                        ]
                        right_data["target"].append([right_last_timer_time] + joints)
                    except ValueError:
                        invalid_lines.append(
                            f"第{line_num}行：right set target关节角度转换失败 - {line_stripped}"
                        )

    except FileNotFoundError:
        raise FileNotFoundError(f"错误：日志文件不存在 - {log_file_path}")
    except Exception as e:
        raise RuntimeError(f"日志读取异常：{str(e)}")

    # 转换为numpy数组并排序
    for robot_data in [left_data, right_data]:
        for data_type in ["get", "set", "target"]:
            if robot_data[data_type]:
                robot_data[data_type] = np.array(
                    robot_data[data_type], dtype=np.float64
                )
                robot_data[data_type] = robot_data[data_type][
                    robot_data[data_type][:, 0].argsort()
                ]
            else:
                robot_data[data_type] = np.array([], dtype=np.float64).reshape(0, 7)

    return left_data, right_data, invalid_lines


def save_to_csv_dual(left_data, right_data, save_dir="./data/"):
    """将双机器人关联数据保存到CSV文件"""
    import os

    os.makedirs(save_dir, exist_ok=True)

    headers = "time(s),joint1(rad),joint2(rad),joint3(rad),joint4(rad),joint5(rad),joint6(rad)"

    for robot_name, robot_data in [("left", left_data), ("right", right_data)]:
        for data_type in ["get", "set", "target"]:
            if robot_data[data_type].shape[0] > 0:
                save_path = f"{save_dir}{robot_name}_aubo_{data_type}_joint_data.csv"
                np.savetxt(
                    save_path,
                    robot_data[data_type],
                    fmt="%.8f",
                    delimiter=",",
                    header=headers,
                    comments="",
                )
                print(
                    f"✅ {robot_name.upper()} {data_type.upper()}数据已保存到CSV：{save_path}"
                )
            else:
                print(f"跳过{robot_name} {data_type}数据CSV保存：无有效数据")


def plot_dual_robot_joint_trends(
    left_data, right_data, save_plot_path="./data/dual_robot_joint_trends.png"
):
    """
    可视化双机器人关节角度随时间变化趋势
    创建2x6子图布局，上排显示左机器人，下排显示右机器人
    """

    # 检查数据
    left_has_data = any(len(data) > 0 for data in left_data.values())
    right_has_data = any(len(data) > 0 for data in right_data.values())

    if not left_has_data and not right_has_data:
        print("跳过可视化：两个机器人都没有有效数据")
        return

    # 创建2x6子图布局
    fig, axes = plt.subplots(2, 6, figsize=(24, 10))
    fig.suptitle(
        "Dual Robot Joint Angles Comparison", fontsize=20, fontweight="bold", y=0.98
    )

    joint_labels = ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"]
    robot_names = ["Left Aubo", "Right Aubo"]
    robot_data_list = [left_data, right_data]

    # 定义颜色
    get_color = "#FB0000"  # 红色 - 实际角度
    set_color = "#0814F4"  # 蓝色 - PID目标角度
    target_color = "#11E72D"  # 绿色 - 期望角度

    for robot_idx, (robot_name, robot_data) in enumerate(
        zip(robot_names, robot_data_list)
    ):
        # 检查当前机器人是否有数据
        has_data = any(len(data) > 0 for data in robot_data.values())

        if not has_data:
            # 如果没有数据，在所有关节位置显示提示
            for joint_idx in range(6):
                axes[robot_idx, joint_idx].text(
                    0.5,
                    0.5,
                    f"No {robot_name} Data",
                    ha="center",
                    va="center",
                    transform=axes[robot_idx, joint_idx].transAxes,
                    fontsize=12,
                    color="red",
                )
                axes[robot_idx, joint_idx].set_title(
                    f"{robot_name} - {joint_labels[joint_idx]}",
                    fontsize=12,
                    fontweight="bold",
                )
            continue

        # 提取时间与各关节数据（弧度转角度）
        get_time = (
            robot_data["get"][:, 0] if len(robot_data["get"]) > 0 else np.array([])
        )
        set_time = (
            robot_data["set"][:, 0] if len(robot_data["set"]) > 0 else np.array([])
        )
        target_time = (
            robot_data["target"][:, 0]
            if len(robot_data["target"]) > 0
            else np.array([])
        )

        get_joints_deg = (
            [robot_data["get"][:, i] * 180 / np.pi for i in range(1, 7)]
            if len(robot_data["get"]) > 0
            else [np.array([]) for _ in range(6)]
        )
        set_joints_deg = (
            [robot_data["set"][:, i] * 180 / np.pi for i in range(1, 7)]
            if len(robot_data["set"]) > 0
            else [np.array([]) for _ in range(6)]
        )
        target_joints_deg = (
            [robot_data["target"][:, i] * 180 / np.pi for i in range(1, 7)]
            if len(robot_data["target"]) > 0
            else [np.array([]) for _ in range(6)]
        )

        # 逐个关节绘制趋势图
        for joint_idx in range(6):
            ax = axes[robot_idx, joint_idx]

            # 绘制各类型数据
            if len(get_time) > 0:
                ax.plot(
                    get_time,
                    get_joints_deg[joint_idx],
                    color=get_color,
                    linewidth=2,
                    marker="o",
                    markersize=2,
                    alpha=0.7,
                    label="Get (Actual)",
                )

            if len(set_time) > 0:
                ax.plot(
                    set_time,
                    set_joints_deg[joint_idx],
                    color=set_color,
                    linewidth=1.5,
                    marker="x",
                    markersize=1.5,
                    alpha=0.6,
                    label="Set (PID Target)",
                )

            if len(target_time) > 0:
                ax.plot(
                    target_time,
                    target_joints_deg[joint_idx],
                    color=target_color,
                    linewidth=1,
                    marker="",
                    markersize=1,
                    alpha=0.5,
                    label="Target (Desired)",
                )

            # 设置子图标题与标签
            ax.set_title(
                f"{robot_name} - {joint_labels[joint_idx]}",
                fontsize=12,
                fontweight="bold",
            )
            ax.set_xlabel("Time (s)", fontsize=10)
            ax.set_ylabel("Angle (deg)", fontsize=10)

            # 添加图例（只在第一个关节添加，避免重复）
            if joint_idx == 0:
                ax.legend(fontsize=8, loc="upper right")

            # 添加网格
            ax.grid(True, alpha=0.3, linestyle="--")

            # 优化坐标轴刻度
            ax.tick_params(axis="both", labelsize=9)

            # 设置时间轴范围
            all_times = []
            if len(get_time) > 0:
                all_times.extend(get_time)
            if len(set_time) > 0:
                all_times.extend(set_time)
            if len(target_time) > 0:
                all_times.extend(target_time)

            if all_times:
                min_time = min(all_times)
                max_time = max(all_times)
                ax.set_xlim(min_time - 0.1, max_time + 0.1)

            # 角度轴自适应范围
            all_angles = []
            if len(get_joints_deg[joint_idx]) > 0:
                all_angles.extend(get_joints_deg[joint_idx])
            if len(set_joints_deg[joint_idx]) > 0:
                all_angles.extend(set_joints_deg[joint_idx])
            if len(target_joints_deg[joint_idx]) > 0:
                all_angles.extend(target_joints_deg[joint_idx])

            if all_angles:
                overall_min = min(all_angles)
                overall_max = max(all_angles)
                range_diff = overall_max - overall_min
                if range_diff > 0.1:
                    ax.set_ylim(
                        overall_min - range_diff * 0.05, overall_max + range_diff * 0.05
                    )
                else:
                    ax.set_ylim(overall_min - 1, overall_max + 1)

    # 调整子图间距
    plt.tight_layout(rect=[0, 0.02, 1, 0.95])

    # 保存图片
    plt.savefig(save_plot_path, dpi=300, bbox_inches="tight", facecolor="white")
    plt.show()
    plt.close()
    print(f"✅ 双机器人趋势图已保存到：{save_plot_path}")


def plot_joint_comparison(
    left_data, right_data, save_plot_path="./data/joint_comparison.png"
):
    """
    绘制左右机器人同一关节的对比图
    创建2x3子图布局，每个子图显示一个关节的左右机器人对比
    """

    # 检查数据
    left_has_data = any(len(data) > 0 for data in left_data.values())
    right_has_data = any(len(data) > 0 for data in right_data.values())

    if not left_has_data and not right_has_data:
        print("跳过关节对比可视化：两个机器人都没有有效数据")
        return

    # 创建2x3子图布局
    fig, axes = plt.subplots(2, 3, figsize=(18, 12))
    fig.suptitle(
        "Left vs Right Robot Joint Angles Comparison",
        fontsize=16,
        fontweight="bold",
        y=0.98,
    )

    joint_labels = ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"]

    for joint_idx in range(6):
        row = joint_idx // 3
        col = joint_idx % 3
        ax = axes[row, col]

        # 提取左机器人数据
        if len(left_data["get"]) > 0:
            left_time = left_data["get"][:, 0]
            left_angle = left_data["get"][:, joint_idx + 1] * 180 / np.pi
            ax.plot(
                left_time, left_angle, "r-", linewidth=2, label="Left Robot", alpha=0.8
            )

        # 提取右机器人数据
        if len(right_data["get"]) > 0:
            right_time = right_data["get"][:, 0]
            right_angle = right_data["get"][:, joint_idx + 1] * 180 / np.pi
            ax.plot(
                right_time,
                right_angle,
                "b-",
                linewidth=2,
                label="Right Robot",
                alpha=0.8,
            )

        ax.set_title(
            f"{joint_labels[joint_idx]} Comparison", fontsize=12, fontweight="bold"
        )
        ax.set_xlabel("Time (s)", fontsize=10)
        ax.set_ylabel("Angle (degrees)", fontsize=10)
        ax.legend(fontsize=10)
        ax.grid(True, alpha=0.3, linestyle="--")
        ax.tick_params(axis="both", labelsize=9)

    plt.tight_layout(rect=[0, 0.02, 1, 0.95])
    plt.savefig(save_plot_path, dpi=300, bbox_inches="tight", facecolor="white")
    plt.show()
    plt.close()
    print(f"✅ 关节对比图已保存到：{save_plot_path}")


def print_dual_robot_stats(left_data, right_data):
    """打印双机器人统计信息"""
    print("\n" + "=" * 80)
    print("📊 Dual Robot Data Analysis Statistics")
    print("=" * 80)

    for robot_name, robot_data in [("LEFT", left_data), ("RIGHT", right_data)]:
        print(f"\n🤖 {robot_name} AUBO Robot:")
        print("-" * 50)

        total_points = sum(len(data) for data in robot_data.values())
        if total_points == 0:
            print(f"❌ No valid data found for {robot_name} robot")
            continue

        for data_type in ["get", "set", "target"]:
            data_array = robot_data[data_type]
            if len(data_array) > 0:
                time_span = (
                    data_array[-1, 0] - data_array[0, 0] if len(data_array) > 1 else 0
                )
                print(
                    f"  {data_type.upper()} data: {len(data_array)} points, duration: {time_span:.1f}s"
                )

                # 打印关节角度范围（度）
                print("    Joint angle ranges (degrees):")
                for i in range(1, min(7, data_array.shape[1])):
                    degrees = np.degrees(data_array[:, i])
                    print(
                        f"      Joint {i}: {degrees.min():.1f}° ~ {degrees.max():.1f}°"
                    )
            else:
                print(f"  {data_type.upper()} data: No data")

        # 计算统计信息
        if len(robot_data["get"]) > 0:
            print("  📈 Motion Statistics:")
            get_data = robot_data["get"]
            if len(get_data) > 1:
                # 计算关节运动范围
                joint_ranges = []
                for i in range(1, 7):
                    joint_angles = np.degrees(get_data[:, i])
                    joint_range = joint_angles.max() - joint_angles.min()
                    joint_ranges.append(joint_range)

                max_range_joint = np.argmax(joint_ranges) + 1
                print(
                    f"    Most active joint: Joint{max_range_joint} (range: {max(joint_ranges):.1f}°)"
                )
                print(
                    f"    Average update frequency: {len(get_data) / ((get_data[-1, 0] - get_data[0, 0]) or 1):.1f} Hz"
                )


def main():
    """主函数"""
    LOG_FILE_PATH = "../build/joint_logs.txt"
    SAVE_DIR = "./data/"
    DUAL_PLOT_SAVE_PATH = "./data/dual_robot_joint_trends.png"
    COMPARISON_PLOT_SAVE_PATH = "./data/joint_comparison.png"

    print("✅ 双机器人关节数据分析脚本启动")

    try:
        # 1. 提取双机器人关节数据
        print("🔍 开始提取双机器人关节数据...")
        left_data, right_data, invalid_lines = extract_joint_and_time_dual_robot(
            LOG_FILE_PATH
        )

        # 2. 输出数据提取概况
        print("\n📊 数据提取结果概况:")
        print("日志文件：{LOG_FILE_PATH}")
        print("过滤无效数据：{len(invalid_lines)} 行")

        # 3. 打印详细统计信息
        print_dual_robot_stats(left_data, right_data)

        # 4. 保存数据到CSV
        print("\n💾 保存数据到CSV文件...")
        save_to_csv_dual(left_data, right_data, SAVE_DIR)

        # 5. 生成双机器人关节趋势对比图
        print("\n🎨 生成双机器人关节趋势对比图...")
        plot_dual_robot_joint_trends(left_data, right_data, DUAL_PLOT_SAVE_PATH)

        # 6. 生成关节对比图
        print("\n🎨 生成左右机器人关节对比图...")
        plot_joint_comparison(left_data, right_data, COMPARISON_PLOT_SAVE_PATH)

        print("\n🎉 双机器人数据分析完成！")

    except Exception as e:
        print(f"❌ 脚本执行出错：{e}")
        import traceback

        traceback.print_exc()


if __name__ == "__main__":
    main()
