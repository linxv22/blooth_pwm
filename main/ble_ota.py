import asyncio
import os
from bleak import BleakScanner, BleakClient

# ----------------- 配置区 -----------------
DEVICE_NAME = "ESP32_BLE"        # 按需要修改为你板子广播的名字
BIN_FILE = "build/blooth_pwm.bin" # 这里指向你想要烧录的固件
CHUNK_SIZE = 200                 # 每包大小，小于等于 MTU(256) - 3 即可

# 我们刚在 ESP32 代码里定义的特征 UUID
UUID_OTA_CTRL = "0000A001-0000-1000-8000-00805F9B34FB"
UUID_OTA_DATA = "0000A002-0000-1000-8000-00805F9B34FB"
# ------------------------------------------

async def main():
    print(f"正在寻找设备: {DEVICE_NAME}...")
    device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=10.0)
    
    if not device:
        print("❌ 未找到设备，请确认板子是否正在广播！")
        return

    print(f"✅ 找到设备: {device.name} ({device.address})")

    if not os.path.exists(BIN_FILE):
        print(f"❌ 找不到固件文件: {BIN_FILE}")
        return
        
    file_size = os.path.getsize(BIN_FILE)
    print(f"📦 准备发送固件大小: {file_size} Bytes")

    async with BleakClient(device) as client:
        print("🔗 蓝牙已连接，准备开始 OTA...")

        # 1. 发送 START 指令 (0x01)
        # 用 bytearray 把 0x01 包装起来发过去
        await client.write_gatt_char(UUID_OTA_CTRL, bytearray([0x01]), response=True)
        print("▶️ START 指令已发送")
        
        # 加上一点延时让 ESP32 初始化 Flash
        await asyncio.sleep(0.5)

        # 2. 分片发送数据
        with open(BIN_FILE, "rb") as f:
            sent_bytes = 0
            while True:
                chunk = f.read(CHUNK_SIZE)
                if not chunk:
                    break
                    
                # 写入数据特征 (注意这里是 False，即 WRITE_NO_RSP，大幅提高速度)
                await client.write_gatt_char(UUID_OTA_DATA, chunk, response=False)
                sent_bytes += len(chunk)
                
                # 打印进度条
                progress = (sent_bytes / file_size) * 100
                print(f"\r🚀 发送进度: [{sent_bytes}/{file_size}] {progress:.1f}%", end="")
                
                # 给板子一点喘息时间写入 Flash (这里可根据实际稳定性微调)
                await asyncio.sleep(0.01) 
                
            print("\n✅ 所有数据块发送完毕。")

        # 3. 发送 END 指令 (0x02)
        await client.write_gatt_char(UUID_OTA_CTRL, bytearray([0x02]), response=True)
        print("⏹️ END 指令已发送，请等待设备重启！")

if __name__ == "__main__":
    asyncio.run(main())