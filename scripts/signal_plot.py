"""
测试 Butterworth 滤波器的频率响应和滤波效果可视化
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
from scipy.fft import fft, fftfreq
import pandas as pd


def load_filter_data(csv_file=None, data_text=None):
    """加载滤波器测试数据"""
    if csv_file:
        # 从CSV文件加载
        data = pd.read_csv(csv_file)
        input_signal = data["Input"].values
        filtered_signal = data["Filtered"].values
    elif data_text:
        # 从文本数据加载
        lines = data_text.strip().split("\n")[1:]  # 跳过标题行
        input_signal = []
        filtered_signal = []
        for line in lines:
            inp, filt = line.split(",")
            input_signal.append(float(inp))
            filtered_signal.append(float(filt))
        input_signal = np.array(input_signal)
        filtered_signal = np.array(filtered_signal)

    return input_signal, filtered_signal


def analyze_frequency_response(input_signal, filtered_signal, fs=100):
    """分析频率响应"""
    N = len(input_signal)

    # 计算FFT
    input_fft = fft(input_signal)
    filtered_fft = fft(filtered_signal)
    freqs = fftfreq(N, 1 / fs)

    # 只取正频率部分
    pos_mask = freqs >= 0
    freqs = freqs[pos_mask]
    input_fft = input_fft[pos_mask]
    filtered_fft = filtered_fft[pos_mask]

    # 计算幅度谱
    input_magnitude = np.abs(input_fft)
    filtered_magnitude = np.abs(filtered_fft)

    # 计算传递函数
    # 避免除零
    transfer_function = np.divide(
        filtered_fft,
        input_fft,
        out=np.zeros_like(filtered_fft),
        where=np.abs(input_fft) > 1e-10,
    )

    return freqs, input_magnitude, filtered_magnitude, transfer_function


def plot_time_domain(input_signal, filtered_signal, fs=100):
    """绘制时域信号"""
    time = np.arange(len(input_signal)) / fs

    plt.figure(figsize=(12, 8))

    # 时域信号对比
    plt.subplot(2, 2, 1)
    plt.plot(time, input_signal, "b-", alpha=0.7, label="Input Signal")
    plt.plot(time, filtered_signal, "r-", linewidth=2, label="Filtered Signal")
    plt.xlabel("Time (s)")
    plt.ylabel("Amplitude")
    plt.title("Time Domain Comparison")
    plt.legend()
    plt.grid(True, alpha=0.3)

    # 输入信号特写
    plt.subplot(2, 2, 2)
    plt.plot(time[:50], input_signal[:50], "b-", label="Input")
    plt.plot(time[:50], filtered_signal[:50], "r-", label="Filtered")
    plt.xlabel("Time (s)")
    plt.ylabel("Amplitude")
    plt.title("Signal Detail (First 0.5s)")
    plt.legend()
    plt.grid(True, alpha=0.3)

    # 滤波效果评估
    plt.subplot(2, 2, 3)
    error = input_signal - filtered_signal
    plt.plot(time, error, "g-", alpha=0.7)
    plt.xlabel("Time (s)")
    plt.ylabel("Error")
    plt.title(f"Filtering Error (RMS: {np.sqrt(np.mean(error**2)):.4f})")
    plt.grid(True, alpha=0.3)

    # 相位延迟分析
    plt.subplot(2, 2, 4)
    # 寻找输入和输出的峰值位置
    input_peaks = signal.find_peaks(input_signal, height=0.5)[0]
    filtered_peaks = signal.find_peaks(filtered_signal, height=0.3)[0]

    if len(input_peaks) > 0 and len(filtered_peaks) > 0:
        phase_delay = (filtered_peaks[0] - input_peaks[0]) / fs * 1000  # ms
        plt.scatter(
            time[input_peaks[:10]],
            input_signal[input_peaks[:10]],
            c="blue",
            s=50,
            label=f"Input Peaks",
        )
        plt.scatter(
            time[filtered_peaks[:10]],
            filtered_signal[filtered_peaks[:10]],
            c="red",
            s=50,
            label=f"Filtered Peaks",
        )
        plt.plot(time, input_signal, "b-", alpha=0.3)
        plt.plot(time, filtered_signal, "r-", alpha=0.3)
        plt.title(f"Phase Delay: {phase_delay:.2f} ms")
    else:
        plt.plot(time, input_signal, "b-", alpha=0.7, label="Input")
        plt.plot(time, filtered_signal, "r-", alpha=0.7, label="Filtered")
        plt.title("Peak Detection Failed")

    plt.xlabel("Time (s)")
    plt.ylabel("Amplitude")
    plt.legend()
    plt.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()


def plot_frequency_domain(
    freqs,
    input_magnitude,
    filtered_magnitude,
    transfer_function,
    filter_order=4,
    cutoff_freq=10,
    fs=1000,
):
    """绘制频域分析"""
    plt.figure(figsize=(15, 10))

    # 幅度谱对比
    plt.subplot(2, 3, 1)
    plt.semilogy(freqs, input_magnitude, "b-", alpha=0.7, label="Input")
    plt.semilogy(freqs, filtered_magnitude, "r-", linewidth=2, label="Filtered")
    plt.xlabel("Frequency (Hz)")
    plt.ylabel("Magnitude")
    plt.title("Frequency Domain Magnitude")
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.xlim(0, fs / 2)  # 只显示0-fs/2范围

    # 传递函数幅度
    plt.subplot(2, 3, 2)
    transfer_magnitude = np.abs(transfer_function)
    transfer_magnitude_db = 20 * np.log10(transfer_magnitude + 1e-10)
    plt.plot(freqs, transfer_magnitude_db, "g-", linewidth=2)
    plt.xlabel("Frequency (Hz)")
    plt.ylabel("Magnitude (dB)")
    plt.title("Filter Transfer Function")
    plt.grid(True, alpha=0.3)
    plt.xlim(0, fs / 2)
    plt.ylim(-60, 5)

    # 相位响应
    plt.subplot(2, 3, 3)
    transfer_phase = np.angle(transfer_function) * 180 / np.pi
    plt.plot(freqs, transfer_phase, "purple", linewidth=2)
    plt.xlabel("Frequency (Hz)")
    plt.ylabel("Phase (degrees)")
    plt.title("Filter Phase Response")
    plt.grid(True, alpha=0.3)
    plt.xlim(0, fs / 2)

    # 理论Butterworth响应对比
    plt.subplot(2, 3, 4)
    # 使用与实际滤波器相同的参数
    b_theory, a_theory = signal.butter(filter_order, cutoff_freq, "low", fs=fs)
    w_theory, h_theory = signal.freqz(b_theory, a_theory, worN=512, fs=fs)
    h_theory_db = 20 * np.log10(np.abs(h_theory))

    transfer_magnitude_db = 20 * np.log10(np.abs(transfer_function) + 1e-10)

    plt.plot(
        w_theory,
        h_theory_db,
        "k--",
        linewidth=2,
        label=f"Theoretical {filter_order}th-order",
    )
    plt.plot(freqs, transfer_magnitude_db, "g-", linewidth=2, label="Actual Filter")
    plt.xlabel("Frequency (Hz)")
    plt.ylabel("Magnitude (dB)")
    plt.title("Comparison with Theoretical Butterworth")
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.xlim(0, fs / 2)
    plt.ylim(-60, 5)

    # 噪声抑制分析
    plt.subplot(2, 3, 5)
    noise_reduction = input_magnitude / (filtered_magnitude + 1e-10)
    noise_reduction_db = 20 * np.log10(noise_reduction)
    plt.plot(freqs, noise_reduction_db, "orange", linewidth=2)
    plt.xlabel("Frequency (Hz)")
    plt.ylabel("Noise Reduction (dB)")
    plt.title("Noise Reduction vs Frequency")
    plt.grid(True, alpha=0.3)
    plt.xlim(0, fs / 2)

    # 频谱密度对比
    plt.subplot(2, 3, 6)
    plt.loglog(freqs, input_magnitude**2, "b-", alpha=0.7, label="Input PSD")
    plt.loglog(freqs, filtered_magnitude**2, "r-", linewidth=2, label="Filtered PSD")
    plt.xlabel("Frequency (Hz)")
    plt.ylabel("Power Spectral Density")
    plt.title("Power Spectral Density")
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.xlim(1, fs / 2)

    plt.tight_layout()
    plt.show()


def analyze_filter_performance(input_signal, filtered_signal, fs=100):
    """分析滤波器性能指标"""
    print("=== 滤波器性能分析 ===")

    # 基本统计信息
    print(f"信号长度: {len(input_signal)} 采样点")
    print(f"采样频率: {fs} Hz")
    print(f"信号时长: {len(input_signal) / fs:.2f} 秒")

    # 幅度统计
    print(f"\n=== 幅度统计 ===")
    print(
        f"输入信号 - 均值: {np.mean(input_signal):.4f}, 标准差: {np.std(input_signal):.4f}"
    )
    print(
        f"滤波信号 - 均值: {np.mean(filtered_signal):.4f}, 标准差: {np.std(filtered_signal):.4f}"
    )
    print(
        f"峰值衰减: {(np.max(input_signal) - np.max(filtered_signal)) / np.max(input_signal) * 100:.2f}%"
    )

    # 误差分析
    error = input_signal - filtered_signal
    mse = np.mean(error**2)
    rmse = np.sqrt(mse)
    snr = 20 * np.log10(np.std(input_signal) / rmse)

    print(f"\n=== 误差分析 ===")
    print(f"均方误差 (MSE): {mse:.6f}")
    print(f"均方根误差 (RMSE): {rmse:.6f}")
    print(f"信噪比 (SNR): {snr:.2f} dB")

    # 频率分析
    freqs, input_mag, filtered_mag, _ = analyze_frequency_response(
        input_signal, filtered_signal, fs
    )

    # 找到主要频率成分
    input_peak_idx = np.argmax(input_mag[1:]) + 1  # 跳过DC分量
    filtered_peak_idx = np.argmax(filtered_mag[1:]) + 1

    print(f"\n=== 频率分析 ===")
    print(f"输入信号主频率: {freqs[input_peak_idx]:.2f} Hz")
    print(f"滤波信号主频率: {freqs[filtered_peak_idx]:.2f} Hz")

    # 截止频率估计（-3dB点）
    transfer_mag = filtered_mag / (input_mag + 1e-10)
    transfer_db = 20 * np.log10(transfer_mag + 1e-10)
    cutoff_idx = np.where(transfer_db <= -3)[0]
    if len(cutoff_idx) > 0:
        cutoff_freq = freqs[cutoff_idx[0]]
        print(f"估计截止频率 (-3dB): {cutoff_freq:.2f} Hz")


def main():
    """主函数"""
    # 您提供的测试数据
    # data_file = r"../build/butterworth_filter.csv"
    data_file = r"../build/gaussian_filter_test.csv"
    filter_order = 6
    cutoff_frequency = 20  # Hz
    fs = 1000  # 采样频率
    # 加载数据
    input_signal, filtered_signal = load_filter_data(csv_file=data_file)

    # 分析性能
    analyze_filter_performance(input_signal, filtered_signal, fs=fs)

    # 频率分析
    freqs, input_mag, filtered_mag, transfer_func = analyze_frequency_response(
        input_signal, filtered_signal, fs=fs
    )

    # 绘制图形
    plot_time_domain(input_signal, filtered_signal, fs=fs)
    plot_frequency_domain(
        freqs,
        input_mag,
        filtered_mag,
        transfer_func,
        filter_order,
        cutoff_frequency,
        fs,
    )


if __name__ == "__main__":
    main()
