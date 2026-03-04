import time
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

def main():
    print("🔄 正在初始化 I2C 总线...")
    # 使用你之前成功连通的 I2C 端口
    i2c = busio.I2C(board.SCL_1, board.SDA_1)

    print("🔌 正在连接 PCA9685 (地址 0x41)...")
    pca = PCA9685(i2c, address=0x41)
    
    # 【非常重要】绝大多数常见舵机都需要 50Hz 频率，不要用 60Hz
    pca.frequency = 50  
    print("✅ PCA9685 初始化成功，频率设置为 50Hz")

    # 绑定 PCA9685 驱动板上标号为 "4" 的插槽
    print("🎯 正在初始化 Channel 4 上的舵机...")
    channel_4_servo = servo.Servo(pca.channels[4])

    try:
        print("\n🚀 开始转动测试！请观察舵机...")
        
        print("➡️  转到 0 度")
        channel_4_servo.angle = 0
        time.sleep(2)

        print("⬆️  转到 90 度 (回中)")
        channel_4_servo.angle = 90
        time.sleep(2)

        print("⬅️  转到 180 度")
        channel_4_servo.angle = 180
        time.sleep(2)

        print("⬆️  测试完成，回到 90 度...")
        channel_4_servo.angle = 90
        time.sleep(1)

    except KeyboardInterrupt:
        print("\n🛑 用户手动停止了测试。")
        
    finally:
        # 释放该通道的 PWM 信号，让舵机放松（不再死锁保持力矩）
        pca.channels[4].duty_cycle = 0
        pca.deinit()
        print("🧹 清理完成，程序退出。")

if __name__ == "__main__":
    main()
