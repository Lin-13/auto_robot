import numpy as np
import matplotlib.pyplot as plt


def draw_coordinate_system(ax, T, length=1.0, label=""):
    """简化的坐标系绘制函数，包含坐标系名称显示"""
    # 提取平移和旋转分量
    pos = T[:3, 3]  # 位置
    x_axis = T[:3, 0]  # X轴方向
    y_axis = T[:3, 1]  # Y轴方向
    z_axis = T[:3, 2]  # Z轴方向

    # 绘制坐标轴
    ax.quiver(
        pos[0],
        pos[1],
        pos[2],
        x_axis[0] * length,
        x_axis[1] * length,
        x_axis[2] * length,
        color="r",
        label=f"X{label}" if label else "X",
    )
    ax.quiver(
        pos[0],
        pos[1],
        pos[2],
        y_axis[0] * length,
        y_axis[1] * length,
        y_axis[2] * length,
        color="g",
        label=f"Y{label}" if label else "Y",
    )
    ax.quiver(
        pos[0],
        pos[1],
        pos[2],
        z_axis[0] * length,
        z_axis[1] * length,
        z_axis[2] * length,
        color="b",
        label=f"Z{label}" if label else "Z",
    )

    # 在坐标轴末端添加名称标签
    # 整体坐标系名称（位于原点附近）
    if label:
        ax.text(
            pos[0],
            pos[1],
            pos[2],  # 文本位置（原点）
            label,  # 显示的名称
            color="black",  # 文本颜色
            fontsize=10,
            fontweight="bold",
        )
    # x_end = pos + x_axis * length
    # y_end = pos + y_axis * length
    # z_end = pos + z_axis * length
    # ax.text(x_end[0], x_end[1], x_end[2], "X", color="r", fontsize=8)
    # ax.text(y_end[0], y_end[1], y_end[2], "Y", color="g", fontsize=8)
    # ax.text(z_end[0], z_end[1], z_end[2], "Z", color="b", fontsize=8)

    return ax


def test_draw_coordinate_system():
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection="3d")
    T = np.array([[1, 0, 0, 1.5], [0, 1, 0, 0.8], [0, 0, 1, 0.5], [0, 0, 0, 1]])

    draw_coordinate_system(ax, T, length=1.0, label="A")
    # 设置坐标轴范围和标签
    ax.set_xlim(0, 3)
    ax.set_ylim(0, 3)
    ax.set_zlim(0, 3)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    plt.show()


if __name__ == "__main__":
    test_draw_coordinate_system()
