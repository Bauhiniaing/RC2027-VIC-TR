# CMake 工程用 DAPLink 烧录说明

适用：STM32CubeMX 生成的 **CMake** 工程 + VS Code（ST 官方 STM32 插件）+ **DAPLink（CMSIS-DAP）** 下载器。
烧录工具用 **pyOCD**（ST 插件自带的调试器只支持 ST-Link，DAPLink 要用 pyOCD 或 OpenOCD）。

---

## 一、一次性准备（已完成，换电脑时才需要重做）

### 1. 安装 pyOCD
```bash
python -m pip install -i https://pypi.tuna.tsinghua.edu.cn/simple pyocd
python -m pyocd --version        # 能打印版本号就说明装好了
```
（官方源在国内很慢，所以用清华镜像。）

### 2. 器件包（芯片支持包）
pyOCD 自带的芯片很少，需要用 Keil 官方器件包（.pack），里面有 Flash 烧录算法。
已下载到 `~/tools/packs/`（Windows 资源管理器地址栏输入 `%USERPROFILE%\tools\packs` 即可打开）：

| 系列 | 文件 |
|---|---|
| STM32H7 | `Keil.STM32H7xx_DFP.4.1.3.pack` |
| STM32F4 | `Keil.STM32F4xx_DFP.3.1.1.pack` |
| STM32F1 | `Keil.STM32F1xx_DFP.2.4.1.pack` |

需要其他系列时，从 Keil 下载，地址格式（把 `xx` 换成系列，版本号见 .pdsc 文件）：
```
https://www.keil.com/pack/Keil.STM32G4xx_DFP.pdsc        ← 查看最新版本号
https://www.keil.com/pack/Keil.STM32G4xx_DFP.<版本>.pack   ← 下载
```
> 不要用 `pyocd pack install`，它的在线索引经常找不到 H7 这类芯片。

---

## 二、日常烧录流程

1. **接线**：DAPLink 的 SWDIO、SWCLK、GND 接到板子，板子要供电，DAPLink 插电脑
2. **编译**：VS Code 底部状态栏点 **Build**（或 `F7`），生成 `build/Debug/<项目名>.elf`
3. **烧录**（二选一）：
   - VS Code：`Ctrl+Shift+P` → `Tasks: Run Task` → **Flash (DAPLink)**
   - 终端（在工程根目录）：
     ```bash
     python -m pyocd flash --pack ~/tools/packs/Keil.STM32H7xx_DFP.4.1.3.pack -t stm32h723vgtx -f 4000000 build/Debug/H7_TR.elf
     ```
     PowerShell 里把 `~` 换成 `$HOME`。
4. 终端出现 `Programming...` 和 `programmed xxx bytes` 即成功，程序自动复位运行。

> 改了代码一定要**先 Build 再烧录**，否则烧进去的还是旧程序。

---

## 三、命令参数说明

```
python -m pyocd flash  --pack <器件包>  -t <芯片型号>  -f <速度>  <elf 文件>
```

| 参数 | 含义 | 例子 |
|---|---|---|
| `--pack` | 器件包路径，按芯片系列选 | `~/tools/packs/Keil.STM32F4xx_DFP.3.1.1.pack` |
| `-t` | 芯片具体型号（小写） | `stm32h723vgtx` / `stm32f407igtx` / `stm32f103c8` |
| `-f` | SWD 速度（Hz），连不上就调低 | `4000000`（4MHz），不稳定改 `1000000` |
| 最后一项 | 编译出的 elf 文件 | `build/Debug/H7_TR.elf` |

### 查芯片型号名
```bash
python -m pyocd list --targets --pack ~/tools/packs/Keil.STM32F4xx_DFP.3.1.1.pack | grep -i f407
```

### 常用型号
| 板子 | `--pack` | `-t` |
|---|---|---|
| 达妙 DM-MC02（H723） | `Keil.STM32H7xx_DFP.4.1.3.pack` | `stm32h723vgtx` |
| 大疆 C 板 / A 板（F407IG） | `Keil.STM32F4xx_DFP.3.1.1.pack` | `stm32f407ighx`（C 板，型号以板上芯片丝印为准） |
| 最小系统板 F103C8T6 | `Keil.STM32F1xx_DFP.2.4.1.pack` | `stm32f103c8` |

---

## 四、VS Code 一键烧录任务（tasks.json）

### 方式 A：放在项目里（每个项目复制一份）
文件位置：`<项目文件夹>/.vscode/tasks.json`
```json
{
    "version": "2.0.0",
    "tasks": [
        {
            "label": "Flash (DAPLink)",
            "type": "shell",
            "command": "python -m pyocd flash --pack \"${userHome}/tools/packs/Keil.STM32H7xx_DFP.4.1.3.pack\" -t stm32h723vgtx -f 4000000 build/Debug/${workspaceFolderBasename}.elf",
            "options": { "cwd": "${workspaceFolder}" },
            "group": "build",
            "problemMatcher": []
        }
    ]
}
```
- `${workspaceFolderBasename}` 会自动替换成文件夹名，只要 **CubeMX 项目名 = 文件夹名**（默认就是），复制过去不用改。
- 换芯片系列时只改 `--pack` 和 `-t`。

### 方式 B：用户级任务（所有项目通用，不用复制）
`Ctrl+Shift+P` → `Tasks: Open User Tasks`，把上面 `tasks` 里的内容粘进去。
可以写多个任务（比如 H7 / F4 / F1 各一个，label 区分），烧录时选对应的那个。

---

## 五、新建项目检查清单

- [ ] CubeMX：`Project Manager → Toolchain/IDE` 选 **CMake**
- [ ] 项目路径**不要有中文和空格**
- [ ] `SYS → Debug` 选 **Serial Wire**（不选的话烧一次之后可能就连不上了，见下方"救砖"）
- [ ] VS Code 打开项目文件夹，Build 通过
- [ ] 有 tasks.json（方式 A）或用户级任务（方式 B），芯片型号对得上

---

## 六、常见问题

| 现象 | 原因 / 解决 |
|---|---|
| `No connected debug probes` | DAPLink 没识别。重插、换 USB 线、换口。`python -m pyocd list` 能看到下载器才算正常 |
| `Target type xxx not recognized` | `-t` 型号写错，或 `--pack` 用错系列 / 路径不对 |
| `No such file ... .elf` | 没编译，或 elf 名字和项目名不一致，看 `build/Debug/` 里实际叫什么 |
| 连接失败 / 烧到一半报错 | 降速：`-f 1000000`；检查 GND 是否接好、板子是否供电 |
| 烧过一次后再也连不上 | 多半是 CubeMX 没开 SWD，或程序进了低功耗。**救砖**：加 `-O connect_mode=under-reset`，并把 DAPLink 的 RST 接到板子 NRST；或者按住板子复位键执行烧录命令、开始后松开 |
| 想整片擦除 | `python -m pyocd erase --chip --pack <器件包> -t <型号>` |
| 设备管理器里 DAPLink 显示一个 COM 口 | 那是 DAPLink 自带的串口（VID FAED），**不是**板子的 USB 虚拟串口 |

---

## 七、（以后再配）在线调试
打断点、看变量需要装 VS Code 插件 **Cortex-Debug**，用 pyOCD 作为 GDB Server，并指定芯片的 SVD 文件。到时候再单独配。
