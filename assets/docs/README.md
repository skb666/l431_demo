---
markdown:
  image_dir: /assets/other
  path: /README.md
  ignore_from_front_matter: true
  absolute_image_path: true
export_on_save:
  markdown: true
---

# l431_demo

## 开发环境

硬件：STM32L431RCT6 或者其它有带 DMA 通道 USART 的单片机 + LED 指示灯
系统：Windows、Linux、Mac 均可
软件：STM32CubeMX + Segger Embedded Studio 或 arm-none-eabi 交叉编译工具链

*推荐：[Segger Embedded Studio + STM32CubeMX 跨平台开发环境搭建](https://blog.csdn.net/skb666/article/details/131658780)

```bash
# arm-none-eabi
sudo apt install binutils-arm-none-eabi gcc-arm-none-eabi build-essential
# pipenv
python -m pip install pipenv -U
```

## 生成固件

**克隆仓库**

```bash
git clone https://github.com/skb666/l431_demo.git
```

**编译 bld、app**

用 `Segger Embedded Studio` 打开 `l431_demo.emProject` 后点击 `Build -> Build Solution` 进行编译，或者分别进入各目录进行编译：

```bash
cd l431_demo

# 编译 bld
cd bld
# make -j${nproc}
cmake -G "Unix Makefiles" -H. -Bbuild -DCMAKE_TOOLCHAIN_FILE=toolchains.cmake
cmake --build build -t all -- -j${nproc}
cd -

# 编译 app
cd app
# make -j${nproc}
cmake -G "Unix Makefiles" -H. -Bbuild -DCMAKE_TOOLCHAIN_FILE=toolchains.cmake
cmake --build build -t all -- -j${nproc}
cd -
```

## 下载程序

可以通过 `Segger Embedded Studio` 分别编译下载 bld 和 app 程序，或者：

```bash
cd uart_iap
# 设置虚拟环境
python -m pipenv update
# 合并 bld 和 app
python -m pipenv run python ./merge.py
```

在将 `bld` 和 `app` 合并之后，手动使用 `jlink`、`stlink` 等下载器将 `output.bin` 烧入到单片机 FLASH 地址 `0x08000000` 处

## 升级 BLD

当 MCU 运行 APP 时，可对 BLD 程序进行升级

**串口升级**

在编译完 BLD 后，用 `USB to TTL` 连接 PC 与 MCU

```bash
python -m pipenv run python ./update.py --bld -d COM5 -b 115200 -f ../bld/emStudio/Output/Release/Exe/l431_bld.bin
python -m pipenv run python ./update.py --bld -d /dev/ttyUSB0 -b 115200 -f ../bld/build/l431_bld.bin
```

**i2c 升级**

在编译完 BLD 后，用 [串口转 I2C 工具](https://github.com/skb666/stm32f3discovery_demo)，连接 MCU 与工具的 I2C

工具升级脚本 [update_i2c.py](https://github.com/skb666/stm32f3discovery_demo/blob/main/tools/update_i2c.py)

```bash
python -m pipenv run python ./update.i2c.py --bld -s 0x3c -d COM5 -b 115200 -f ../bld/emStudio/Output/Release/Exe/l431_bld.bin
python -m pipenv run python ./update.i2c.py --bld -s 0x3c -d /dev/ttyUSB0 -b 115200 -f ../bld/build/l431_bld.bin
```

## 升级 APP

当 MCU 运行 BLD 或 APP 时，均可对 APP 程序进行升级

**串口升级**

在编译完 APP 后，用 `USB to TTL` 连接 PC 与 MCU

```bash
python -m pipenv run python ./update.py -d COM5 -b 115200 -f ../app/emStudio/Output/Release/Exe/l431_app.bin
python -m pipenv run python ./update.py -d /dev/ttyUSB0 -b 115200 -f ../app/build/l431_app.bin
```

**i2c 升级**

在编译完 BLD 后，用 [串口转 I2C 工具](https://github.com/skb666/stm32f3discovery_demo)，连接 MCU 与工具的 I2C

工具升级脚本 [update_i2c.py](https://github.com/skb666/stm32f3discovery_demo/blob/main/tools/update_i2c.py)

```bash
python -m pipenv run python ./update.i2c.py -s 0x3c -d COM5 -b 115200 -f ../app/emStudio/Output/Release/Exe/l431_app.bin
python -m pipenv run python ./update.i2c.py -s 0x3c -d /dev/ttyUSB0 -b 115200 -f ../app/build/l431_app.bin
```

## 升级方案

[STM32 备份升级](/update.md)
