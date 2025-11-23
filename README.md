# STM32 UART Firmware Update (FWU) Bootloader Documentation
# I. Overview
This document explains the existing STM32 UART‑based Firmware Update (FWU) bootloader and how the PC host tool was rewritten to follow the MCU’s predefined update flow. The work involves implementing the host‑side .cpp logic, adding a SysTick‑based timeout, and adjusting the update process so firmware is first written to external flash before being copied into internal flash.

# II. Bootloader Architecture
| Region             | Address Range              | Description                          |
|--------------------|----------------------------|--------------------------------------|
| Bootloader         | 0x08000000 – 0x08001FFF    | 8 KB region, protected, runs on boot |
| BSF                | 0x08002000 – 0x08002FFF    | 4 KB, stores boot flags & headers    |
| Application FW     | 0x08003000                 | Main firmware                        |
| SRAM               | 0x20000000 – 0x20003FFF    | 16 KB runtime RAM                    |

# II. Firmware Update Flow (MCU)
### 1. Boot Stage
![transfer-firmware](assets/images/fwu_mcu_1.png)

### 2. Handshake Stage and Meta Stage
![transfer-firmware](assets/images/fwu_mcu_2.png)

### 3. Transfer Firmware Stage
![transfer-firmware](assets/images/fwu_mcu_3.png)

### 4. Final Checksum Stage
![transfer-firmware](assets/images/fwu_mcu_4.png)

# III. Bootloader Flow (MCU)

### 1. Boot Entry & Boot Flag Check
![boot-entry](assets/images/boot_mcu_1.png)

### 2. External → Internal Flash Update Flow
![boot-entry](assets/images/boot_mcu_2.png)
![boot-entry](assets/images/boot_mcu_3.png)

### 3. After Flashing (Checksum & Reset)
![boot-entry](assets/images/boot_mcu_4.png)

# IV. Host-side Flow (PC)
### 1. Initialization (UART Open) and Send FWU Command
![boot-entry](assets/images/fwu_host_1.png)

### 2. Handshake Flow
![boot-entry](assets/images/fwu_host_2.png)

### 3. Send Metadata (Header)
![boot-entry](assets/images/fwu_host_3.png)

### 4. Transfer Firmware (Chunk Loop)
![boot-entry](assets/images/fwu_host_4.png)

### 5. Final Checksum Command
![boot-entry](assets/images/fwu_host_5.png)

# V. Protocol Specification
| Field | Size (bytes) | Value / Range | Description |
|-------|--------------:|---------------|-------------|
| SOP   | 1             | 0xEF          | Start of packet |
| LEN   | 1             | 0..254        | Number of DATA bytes |
| DATA  | LEN           | —             | Payload (CMD + payload) |
| FCS   | 1             | XOR           | LEN ^ data[0] ^ data[1] ... (simple XOR) |

# VI. Struct
### Host

### Meta

### Frame
