import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
import glob
import sys

def plot_angle_comparison(tx_file=None, rx_file=None):
    """
    绘制TX和RX角度对比图（简洁版）
    
    Parameters:
    -----------
    tx_file : str, TX数据文件路径
    rx_file : str, RX数据文件路径
    """
    
    # 如果没有指定文件，使用最新的文件
    if tx_file is None or rx_file is None:
        # 获取当前目录下的所有csv文件
        data_dir = "../figures_data/data"
        if not os.path.exists(data_dir):
            print(f"目录不存在: {data_dir}")
            return
        
        tx_files = sorted(glob.glob(os.path.join(data_dir, "*TX*.csv")))
        rx_files = sorted(glob.glob(os.path.join(data_dir, "*RX*.csv")))
        
        if not tx_files:
            print("找不到TX数据文件")
            return
        if not rx_files:
            print("找不到RX数据文件")
            return
        
        # 使用最新的文件
        tx_file = tx_files[-1]
        rx_file = rx_files[-1]
        
        print(f"使用TX文件: {os.path.basename(tx_file)}")
        print(f"使用RX文件: {os.path.basename(rx_file)}")
    
    # 读取数据
    try:
        tx_data = pd.read_csv(tx_file)
        rx_data = pd.read_csv(rx_file)
    except Exception as e:
        print(f"读取文件失败: {e}")
        return
    
    # 查找角度列
    tx_angle_col = None
    rx_angle_col = None
    
    for col in tx_data.columns:
        if 'angle' in col.lower() or 'Angle' in col:
            tx_angle_col = col
            break
    
    for col in rx_data.columns:
        if 'angle' in col.lower() or 'Angle' in col:
            rx_angle_col = col
            break
    
    if tx_angle_col is None and len(tx_data.columns) >= 2:
        tx_angle_col = tx_data.columns[1]
    
    if rx_angle_col is None and len(rx_data.columns) >= 2:
        rx_angle_col = rx_data.columns[1]
    
    if tx_angle_col is None or rx_angle_col is None:
        print("无法识别角度列")
        return
    
    # 提取数据
    tx_timestamps = tx_data['Timestamp'] if 'Timestamp' in tx_data.columns else tx_data.iloc[:, 0]
    tx_angles = tx_data[tx_angle_col]
    
    rx_timestamps = rx_data['Timestamp'] if 'Timestamp' in rx_data.columns else rx_data.iloc[:, 0]
    rx_angles = rx_data[rx_angle_col]
    
    # 计算最值
    tx_min, tx_max = tx_angles.min(), tx_angles.max()
    rx_min, rx_max = rx_angles.min(), rx_angles.max()
    
    print(f"TX角度 - 最小值: {tx_min:.4f}, 最大值: {tx_max:.4f}")
    print(f"RX角度 - 最小值: {rx_min:.4f}, 最大值: {rx_max:.4f}")
    
    # 创建图形
    plt.figure(figsize=(12, 6))
    
    # 绘制TX角度曲线 - 蓝色实线
    plt.plot(tx_timestamps, tx_angles, 
             color='blue', linewidth=2.0, 
             label=f'TX Angle (min={tx_min:.2f}, max={tx_max:.2f})')
    
    # 绘制RX角度曲线 - 红色实线
    plt.plot(rx_timestamps, rx_angles, 
             color='red', linewidth=2.0, 
             label=f'RX Angle (min={rx_min:.2f}, max={rx_max:.2f})')
    
    # 设置图形属性
    plt.xlabel('Timestamp (s)', fontsize=12)
    plt.ylabel('Angle (rad)', fontsize=12)
    plt.title('TX vs RX Angle Comparison', fontsize=14, fontweight='bold')
    plt.grid(True, linestyle='--', alpha=0.5)
    
    # 设置图例 - 不显示边框，位置在右上角
    plt.legend(loc='upper right', fontsize=11, frameon=False)
    
    # 调整布局
    plt.tight_layout()
    
    # 保存图形
    save_path = "../figures_data/figures/motor5/testPD_.png"
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    print(f"\n图形已保存至: {save_path}")
    
    # 显示图形
    plt.show()

def plot_simple_comparison(tx_file=None, rx_file=None):
    """
    极简版本 - 只有曲线，图例也不显示最值
    """
    
    if tx_file is None or rx_file is None:
        data_dir = "../figures_data/data"
        if not os.path.exists(data_dir):
            return
        
        tx_files = sorted(glob.glob(os.path.join(data_dir, "*TX*.csv")))
        rx_files = sorted(glob.glob(os.path.join(data_dir, "*RX*.csv")))
        
        if not tx_files or not rx_files:
            return
        
        tx_file = tx_files[-1]
        rx_file = rx_files[-1]
    
    try:
        tx_data = pd.read_csv(tx_file)
        rx_data = pd.read_csv(rx_file)
    except:
        return
    
    # 提取数据
    if len(tx_data.columns) >= 2:
        tx_timestamps = tx_data.iloc[:, 0]
        tx_angles = tx_data.iloc[:, 1]
    
    if len(rx_data.columns) >= 2:
        rx_timestamps = rx_data.iloc[:, 0]
        rx_angles = rx_data.iloc[:, 1]
    
    # 创建图形
    plt.figure(figsize=(10, 5))
    
    # 绘制曲线
    plt.plot(tx_timestamps, tx_angles, 'b-', linewidth=2.0, label='TX Angle')
    plt.plot(rx_timestamps, rx_angles, 'r-', linewidth=2.0, label='RX Angle')
    
    # 基本设置
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (rad)')
    plt.title('Angle Comparison')
    plt.grid(True, alpha=0.3)
    plt.legend()
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    # 使用方法1: 绘制最新的文件对
    if len(sys.argv) == 1:
        print("绘制最新的数据文件...")
        plot_angle_comparison()
    
    # 使用方法2: 指定文件路径
    elif len(sys.argv) == 3:
        tx_file = sys.argv[1]
        rx_file = sys.argv[2]
        plot_angle_comparison(tx_file, rx_file)
    
    else:
        print("使用方法:")
        print("1. python test.py                     # 绘制最新的文件")
        print("2. python test.py tx_file.csv rx_file.csv  # 绘制指定文件")