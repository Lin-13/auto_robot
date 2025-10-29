import re
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rcParams
import warnings

warnings.filterwarnings("ignore")  # 忽略matplotlib无关警告

# 配置中文字体（避免中文乱码）
rcParams["font.sans-serif"] = ["DejaVu Sans", "SimHei", "Arial Unicode MS"]
rcParams["axes.unicode_minus"] = False  # 解决负号显示异常


def extract_joint_and_time(log_file_path):
    """
    从日志文件提取关节状态向量与对应的timer_cb时间，返回关联数据

    参数:
        log_file_path (str): 日志文件路径

    返回:
        np.ndarray: 关联数据数组，形状为(n_valid, 7)，列顺序：time, joint1, joint2, joint3, joint4, joint5, joint6
        list: 过滤的无效数据行（调试用）
    """
    # 正则表达式：匹配timer_cb时间（捕获时间数值）
    timer_time_pattern = re.compile(
        r"aubo right_aubo: timer_cb at\s+([-+]?\d+\.?\d*e?[-+]?\d*)\s+s"
    )
    # 正则表达式：匹配get joint state（捕获6个关节角度）
    get_joint_pattern = re.compile(
        r"aubo right_aubo: get joint state\s+([-+]?\d+\.\d+)\s+([-+]?\d+\.?\d*e?[-+]?\d*)\s+([-+]?\d+\.\d+)\s+([-+]?\d+\.\d+)\s+([-+]?\d+\.\d+)\s+([-+]?\d+\.?\d*e?[-+]?\d*)"
    )
    # 新增：匹配set joint state（捕获6个关节角度）
    set_joint_pattern = re.compile(
        r"aubo right_aubo: set joint state\s+([-+]?\d+\.\d+)\s+([-+]?\d+\.?\d*e?[-+]?\d*)\s+([-+]?\d+\.\d+)\s+([-+]?\d+\.\d+)\s+([-+]?\d+\.\d+)\s+([-+]?\d+\.?\d*e?[-+]?\d*)"
    )
    set_target_pattern = re.compile(
        r"aubo right_aubo: set target\s+([-+]?\d+\.\d+)\s+([-+]?\d+\.?\d*e?[-+]?\d*)\s+([-+]?\d+\.\d+)\s+([-+]?\d+\.\d+)\s+([-+]?\d+\.\d+)\s+([-+]?\d+\.?\d*e?[-+]?\d*)"
    )
    get_data_list = []  # 存储get关节数据（时间+6关节）
    set_data_list = []  # 存储set关节数据（时间+6关节）
    set_target_data_list = []  # 存储set目标关节数据（时间+6关节）
    invalid_lines = []
    last_timer_time = None  # 记录上一次获取的timer_cb时间（关联后续关节状态）

    try:
        with open(log_file_path, "r", encoding="utf-8") as f:
            for line_num, line in enumerate(f, 1):
                line_stripped = line.strip()

                # 1. 提取timer_cb时间（优先更新时间，确保后续关节状态能关联最新时间）
                time_match = timer_time_pattern.search(line_stripped)
                if time_match:
                    try:
                        last_timer_time = float(time_match.group(1))
                    except ValueError:
                        invalid_lines.append(
                            f"第{line_num}行：timer_cb时间转换失败 - {line_stripped}"
                        )
                    continue  # 时间行无需继续匹配关节状态

                # 2. 提取get关节状态，并关联最近一次的timer_cb时间
                get_match = get_joint_pattern.search(line_stripped)
                if get_match:
                    if last_timer_time is None:
                        invalid_lines.append(
                            f"第{line_num}行：get关节状态无对应timer_cb时间 - {line_stripped}"
                        )
                        continue
                    try:
                        joints = [float(get_match.group(i)) for i in range(1, 7)]
                        get_data_list.append([last_timer_time] + joints)
                    except ValueError:
                        invalid_lines.append(
                            f"第{line_num}行：get关节角度转换失败 - {line_stripped}"
                        )

                # 3. 提取set关节状态，并关联最近一次的timer_cb时间（核心修复：复用时间关联逻辑）
                set_match = set_joint_pattern.search(line_stripped)
                if set_match:
                    if last_timer_time is None:
                        invalid_lines.append(
                            f"第{line_num}行：set关节状态无对应timer_cb时间 - {line_stripped}"
                        )
                        continue
                    try:
                        joints = [float(set_match.group(i)) for i in range(1, 7)]
                        set_data_list.append([last_timer_time] + joints)
                    except ValueError:
                        invalid_lines.append(
                            f"第{line_num}行：set关节角度转换失败 - {line_stripped}"
                        )
                # 4. 提取set目标关节状态，并关联最近一次的timer_cb时间（核心修复：复用时间关联逻辑）
                set_target_match = set_target_pattern.search(line_stripped)
                if set_target_match:
                    if last_timer_time is None:
                        invalid_lines.append(
                            f"第{line_num}行：set目标关节状态无对应timer_cb时间 - {line_stripped}"
                        )
                        continue
                    try:
                        joints = [float(set_target_match.group(i)) for i in range(1, 7)]
                        set_target_data_list.append([last_timer_time] + joints)
                    except ValueError:
                        invalid_lines.append(
                            f"第{line_num}行：set目标关节角度转换失败 - {line_stripped}"
                        )
                        continue

    except FileNotFoundError:
        raise FileNotFoundError(f"错误：日志文件不存在 - {log_file_path}")
    except Exception as e:
        raise RuntimeError(f"日志读取异常：{str(e)}")

    # 处理get数据：转换为数组并按时间排序
    get_data_array = np.array([], dtype=np.float64).reshape(0, 7)
    if get_data_list:
        get_data_array = np.array(get_data_list, dtype=np.float64)
        get_data_array = get_data_array[get_data_array[:, 0].argsort()]  # 按时间升序

    # 处理set数据：转换为数组并按时间排序（核心修复：正确生成set数据数组）
    set_data_array = np.array([], dtype=np.float64).reshape(0, 7)
    if set_data_list:
        set_data_array = np.array(set_data_list, dtype=np.float64)
        set_data_array = set_data_array[set_data_array[:, 0].argsort()]  # 按时间升序
        # 处理set目标数据：转换为数组并按时间排序（核心修复：正确生成set目标数据数组）
    set_target_data_array = np.array([], dtype=np.float64).reshape(0, 7)
    if set_target_data_list:
        set_target_data_array = np.array(set_target_data_list, dtype=np.float64)
        set_target_data_array = set_target_data_array[
            set_target_data_array[:, 0].argsort()
        ]  # 按时间升序

    # 输出警告信息
    if not get_data_list:
        print("警告：未提取到有效get关节数据（时间+关节角度）")
    if not set_data_list:
        print("警告：未提取到有效set关节数据（时间+关节角度）")
    if not set_target_data_list:
        print("警告：未提取到有效set目标关节数据（时间+关节角度）")

    return get_data_array, set_data_array, set_target_data_array, invalid_lines


def save_to_csv(data_array, save_path, data_type="get"):
    """将关联数据保存到CSV文件"""
    if data_array.shape[0] == 0:
        print(f"跳过{data_type}数据CSV保存：无有效数据")
        return

    # CSV表头：时间 + 6个关节
    headers = "time(s),joint1(rad),joint2(rad),joint3(rad),joint4(rad),joint5(rad),joint6(rad)"
    np.savetxt(
        save_path,
        data_array,
        fmt="%.8f",  # 保留8位小数，平衡精度与可读性
        delimiter=",",
        header=headers,
        comments="",  # 去除表头默认的#注释
    )
    print(f"✅ {data_type.upper()}数据已保存到CSV：{save_path}")


def plot_joint_trends(
    get_data_array,
    set_data_array,
    target_data_array,
    save_plot_path="./joint_trends.png",
    title="Aubo",
):
    """
    可视化关节角度随时间变化趋势，对比get和set的结果
    生成2x3子图（6个关节各占一个子图），支持保存图片
    """
    # 修复：补充title参数默认值（原代码调用时传了4个参数，但函数定义仅3个，导致参数不匹配）
    if get_data_array.shape[0] < 2 or set_data_array.shape[0] < 2:
        print("跳过可视化：有效数据不足（需至少2个时间点）")
        return

    # 提取时间与各关节数据（弧度转角度，便于直观阅读）
    get_time = get_data_array[:, 0]
    set_time = set_data_array[:, 0]
    target_time = target_data_array[:, 0]
    joint_labels = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    # 弧度转角度（×180/π）
    get_joints_deg = [get_data_array[:, i] * 180 / np.pi for i in range(1, 7)]
    set_joints_deg = [set_data_array[:, i] * 180 / np.pi for i in range(1, 7)]
    target_joints_deg = [target_data_array[:, i] * 180 / np.pi for i in range(1, 7)]

    # 创建2x3子图布局（适合6个关节的趋势展示）
    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    fig.suptitle(title, fontsize=16, fontweight="bold", y=0.98)

    # 定义颜色列表（区分get和set，美观易读）
    get_colors = ["#2E86AB", "#A23B72", "#F18F01", "#C73E1D", "#7209B7", "#02C39A"]
    set_colors = ["#55A8DD", "#C060A1", "#F9B252", "#E06041", "#9D4EDD", "#40D3AC"]
    target_colors = [
        "#1D3341",
        "#311D2B",
        "#574022",
        "#4E291F",
        "#31263A",
        "#204138",
    ]
    get_colors = ["#FB0000"] * 6
    set_colors = ["#0814F4"] * 6
    target_colors = ["#11E72D"] * 6
    # 逐个关节绘制趋势图
    for idx, (
        ax,
        get_joint,
        set_joint,
        target_joint,
        label,
        get_color,
        set_color,
    ) in enumerate(
        zip(
            axes.flat,
            get_joints_deg,
            set_joints_deg,
            target_joints_deg,
            joint_labels,
            get_colors,
            set_colors,
        )
    ):
        # 绘制get折线图
        ax.plot(
            get_time,
            get_joint,
            color=get_color,
            linewidth=2,
            marker="o",
            markersize=3,
            alpha=0.2,
            label="Get (Actual Angle)",
        )
        # 绘制set折线图
        ax.plot(
            set_time,
            set_joint,
            color=set_color,
            linewidth=1,
            marker="x",
            markersize=2,
            alpha=0.3,
            label="Set (PID Target Angle)",
        )
        # 绘制target折线图
        ax.plot(
            target_time,
            target_joint,
            color=target_colors[idx],
            linewidth=1,
            marker="",
            markersize=1,
            alpha=0.3,
            label="Target (Desired Angle)",
        )

        # 设置子图标题与标签
        ax.set_title(f"{label} Joint Angle", fontsize=12, fontweight="bold", pad=10)
        ax.set_xlabel("Time (s)", fontsize=10)
        ax.set_ylabel("Angle (deg)", fontsize=10)

        # 添加图例与网格
        ax.legend(fontsize=9)
        ax.grid(True, alpha=0.3, linestyle="--")

        # 优化坐标轴刻度
        ax.tick_params(axis="both", labelsize=9)
        min_time = min(get_time.min(), set_time.min())
        max_time = max(get_time.max(), set_time.max())
        ax.set_xlim(min_time - 0.05, max_time + 0.05)  # 时间轴留边距

        # 角度轴自适应范围
        overall_min = min(get_joint.min(), set_joint.min())
        overall_max = max(get_joint.max(), set_joint.max())
        range_diff = overall_max - overall_min
        if range_diff > 0.1:
            ax.set_ylim(
                overall_min - range_diff * 0.05, overall_max + range_diff * 0.05
            )
        else:
            ax.set_ylim(overall_min - 0.05, overall_max + 0.05)

    # 调整子图间距（避免重叠）
    plt.tight_layout(rect=[0, 0.02, 1, 0.95])  # 为标题留出空间

    # 保存图片（高分辨率）
    plt.savefig(save_plot_path, dpi=300, bbox_inches="tight", facecolor="white")
    plt.close()
    print(f"✅ 趋势图已保存到：{save_plot_path}")


def main():
    # -------------------------- 配置参数（可根据实际情况修改） --------------------------
    LOG_FILE_PATH = "./build/aubo_test.txt"
    LOG_FILE_PATH = "./build/joint_log.txt"
    GET_CSV_SAVE_PATH = "./scripts/data/get_joint_time_data.csv"  # get数据CSV路径
    SET_CSV_SAVE_PATH = "./scripts/data/set_joint_time_data.csv"  # set数据CSV路径
    SET_TARGET_SAVE_PATH = (
        "./scripts/data/set_joint_target_time_data.csv"  # set目标数据CSV路径
    )
    PLOT_SAVE_PATH = "./scripts/data/joint_time_trends.png"  # 可视化图片路径
    # ----------------------------------------------------------------------------------
    print("✅ 脚本已启动，进入 main 函数")
    try:
        # 1. 提取get和set关节数据（修复：合并提取逻辑，避免重复读取文件）
        print("🔍 开始提取get关节数据（时间+关节角度）...")
        print("🔍 开始提取set关节数据（时间+关节角度）...")
        get_data_array, set_data_array, set_target_data_array, invalid_lines = (
            extract_joint_and_time(LOG_FILE_PATH)
        )

        # 2. 输出数据提取概况
        print("\n" + "=" * 70)
        print("📊 Aubo机器人数据提取结果概况")
        print("=" * 70)
        print(f"日志文件：{LOG_FILE_PATH}")
        print(f"有效get数据：{get_data_array.shape[0]} 组（时间+6关节）")
        print(f"有效set数据：{set_data_array.shape[0]} 组（时间+6关节）")
        print(f"过滤无效数据：{len(invalid_lines)} 行")

        # 3. 预览get前5组数据（若有）
        if get_data_array.shape[0] > 0:
            print("\n📋 get前5组数据预览（时间+关节角度，单位：s/rad）：")
            print(
                f"{'时间(s)':<12} {'关节1':<12} {'关节2':<12} {'关节3':<12} {'关节4':<12} {'关节5':<12} {'关节6':<12}"
            )
            print("-" * 84)
            for i in range(min(5, get_data_array.shape[0])):
                row = get_data_array[i]
                print(
                    f"{row[0]:<12.6f} {row[1]:<12.6f} {row[2]:<12.6f} {row[3]:<12.6f} {row[4]:<12.6f} {row[5]:<12.6f} {row[6]:<12.6f}"
                )

        # 4. 预览set前5组数据（若有）
        if set_data_array.shape[0] > 0:
            print("\n📋 set前5组数据预览（时间+关节角度，单位：s/rad）：")
            print(
                f"{'时间(s)':<12} {'关节1':<12} {'关节2':<12} {'关节3':<12} {'关节4':<12} {'关节5':<12} {'关节6':<12}"
            )
            print("-" * 84)
            for i in range(min(5, set_data_array.shape[0])):
                row = set_data_array[i]
                print(
                    f"{row[0]:<12.6f} {row[1]:<12.6f} {row[2]:<12.6f} {row[3]:<12.6f} {row[4]:<12.6f} {row[5]:<12.6f} {row[6]:<12.6f}"
                )

        # 5. 保存数据到CSV
        save_to_csv(get_data_array, GET_CSV_SAVE_PATH, data_type="get")
        save_to_csv(set_data_array, SET_CSV_SAVE_PATH, data_type="set")
        save_to_csv(set_target_data_array, SET_TARGET_SAVE_PATH, data_type="set_target")

        # 6. 可视化get和set关节变化趋势对比
        print("\n🎨 开始生成get与set关节角度趋势对比图...")
        plot_joint_trends(
            get_data_array,
            set_data_array,
            set_target_data_array,
            PLOT_SAVE_PATH,
            title="Aubo Joints trends",
        )
        print("\n🎉 脚本执行完成！")

    except Exception as e:
        print(f"❌ 脚本执行出错：{e}")


if __name__ == "__main__":
    main()
