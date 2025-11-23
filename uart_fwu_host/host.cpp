#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <string.h>
#include <stdio.h>
#include <cstring>
#include "host.h"

using namespace std;

int fd = -1;
struct termios tty;
int flag = 0;
Uart_Boot_Send_State state = NO_SEND;
// Tính FCS giống MCU
uint8_t calc_fcs_pc(const uint8_t *data, uint8_t len)
{
	uint8_t fcs = len;
	for (size_t i = 0; i < len; i++)
		fcs ^= data[i];
	return fcs;
}
void uartInit(const char *uart_dev = "/dev/ttyUSB0")
{
	fd = open(uart_dev, O_RDWR | O_NOCTTY);
	if (fd < 0)
	{
		perror("open");
		return;
	}

	memset(&tty, 0, sizeof tty);
	if (tcgetattr(fd, &tty) != 0)
	{
		perror("tcgetattr");
		return;
	}

	cfsetospeed(&tty, B115200);
	cfsetispeed(&tty, B115200);

	tty.c_cflag |= (CLOCAL | CREAD);
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8;
	tty.c_cflag &= ~PARENB;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	tty.c_iflag &= ~(IXON | IXOFF | IXANY);
	tty.c_oflag &= ~OPOST;

	tcsetattr(fd, TCSANOW, &tty);
}
void close_uart(int fd)
{
	tcflush(fd, TCIOFLUSH);
	tcdrain(fd);

	int flags = TIOCM_DTR | TIOCM_RTS;
	ioctl(fd, TIOCMBIC, &flags);

	close(fd);

	usleep(100000); // 100ms cho driver ổn định
}
void calculateFirmwareChecksum(const char *filename, firmware_header_t &header)
{
	FILE *f = fopen(filename, "rb");
	if (!f)
	{
		std::cerr << "Cannot open firmware file for checksum calculation\n";
		return;
	}

	fseek(f, 0, SEEK_END);
	size_t size = ftell(f);
	fseek(f, 0, SEEK_SET);

	uint8_t *buf = new uint8_t[size];
	fread(buf, 1, size, f);

	fclose(f);

	// calculate checksum
	uint32_t sum = 0;
	for (size_t i = 0; i < size; i++)
		sum += buf[i];

	header.bin_len = size;
	header.checksum = sum & 0xFFFF;

	delete[] buf;
}

void readuart(uint8_t ch)
{

	if (ch == '\r' || ch == '\n')
	{
		std::cout << std::endl; // xuống hàng
	}
	else if (ch >= 32 && ch <= 126) // ký tự printable
	{
		std::cout << (char)ch; // in liền
	}
	else
	{
		std::cout << "0x" << std::hex << (int)ch << std::dec; // in hex liền
	}
}
bool waitForAck(int fd)
{
	uint8_t ch;

	while (true)
	{
		int n = read(fd, &ch, 1);

		if (n > 0)
		{

			if (ch == UART_BOOT_FWU_ACK) // ACK
			{
				return true;
			}
			else if (ch == UART_BOOT_FWU_READY) // READY
			{
				return true;
			}
			else if (ch == UART_BOOT_FWU_NACK) // NACK
			{
				return false;
			}
			readuart(ch);
		}
	}
}

void printProgress(size_t current, size_t total)
{
	const int barWidth = 50; // độ dài thanh progress
	float ratio = (float)current / total;
	int pos = ratio * barWidth;

	std::cout << "[";
	for (int i = 0; i < barWidth; i++)
	{
		if (i < pos)
			std::cout << "=";
		else if (i == pos)
			std::cout << ">";
		else
			std::cout << " ";
	}
	std::cout << "] " << int(ratio * 100.0) << " %\r";
	std::cout.flush();
}
bool sendByteAndWaitACK(uint8_t byte)
{

	for (int attempt = 1; attempt <= MAX_RETRY; attempt++)
	{
		usleep(3000);
		write(fd, &byte, 1);
		tcdrain(fd);

		uint8_t ch;
		int dot_count = 0;

		if (waitForAck(fd))
		{
			return true;
		}
		if(state == SEND_FCS_STATE)
		{
			return false;
		}
		std::cout << "Retrying to send byte (attempt " << attempt << ")...\n";
	}

	std::cout << "ERROR: byte failed too many retries!\n";
	return false;
}

bool sendFrameWithACK(const uint8_t *data, uint8_t len, uint8_t sop = 0xEF)
{
	// 1. SOP
	if (!sendByteAndWaitACK(sop))
		return false;

	// 2. LEN
	if (!sendByteAndWaitACK(len))
		return false;

	// 3. DATA
	for (uint8_t i = 0; i < len; i++)
	{
		write(fd, &data[i], 1);
		usleep(1);
	}

	if (!waitForAck(fd))
	{
		return false;
	}

	// 4. FCS
	// uint8_t fcs = calc_fcs_pc(data, len);
	uint8_t fcs = calc_fcs_pc(data, len);
	if (!sendByteAndWaitACK(fcs))
		return false;
	return true;
}

bool sendFrameRetry(const uint8_t *data, uint16_t len)
{
	const int MAX_FRAME_RETRY = 3;

	for (int attempt = 1; attempt <= MAX_FRAME_RETRY; attempt++)
	{
		std::cout << "FRAME TRY " << attempt << "/" << MAX_FRAME_RETRY << "\n";

		if (sendFrameWithACK(data, len))
			return true;

		std::cout << "Frame failed, retrying...\n";
	}

	std::cout << "FRAME FAILED after retries!\n";
	return false;
}
bool sendReadySignal()
{
	cout << "SEND UPDATE FW CMD" << endl;
	write(fd, "fwu\r\n", 5);
	uint8_t ch;
	cout << "WAITING ACK" << endl;

	if (!waitForAck(fd))
	{
		return false;
	}
	return true;
}
int main()
{

	uartInit("/dev/ttyUSB0");
	// SEND UPDATE FW CMD
	for (int i = 0; i < 3; i++)
	{
		if (sendReadySignal())
		{
			break;
		}
		cout << "RESEND UPDATE FW CMD: " << i + 1 << endl;
	}

	firmware_boot_cmd_t fw_boot_cmd{};
	fw_boot_cmd.cmd = SYS_BOOT_CMD_NONE;					   // 0x01
	fw_boot_cmd.container = SYS_BOOT_CONTAINER_EXTERNAL_FLASH; // giả định container type
	// fw_boot_cmd.container = SYS_BOOT_CONTAINER_INTERNAL_FLASH; // giả định container type
	fw_boot_cmd.io_driver = 0x02;							   // UART
	fw_boot_cmd.des_addr = 0x000000;
	// fw_boot_cmd.des_addr = APP_START;
	fw_boot_cmd.src_addr = 0;

	fw_boot_cmd.ak_msg_res.type = 0xAA;
	fw_boot_cmd.ak_msg_res.src_task_id = 0x11;
	fw_boot_cmd.ak_msg_res.des_task_id = 0x22;
	fw_boot_cmd.ak_msg_res.sig = 0x33;
	fw_boot_cmd.ak_msg_res.if_src_type = 0x44;
	fw_boot_cmd.ak_msg_res.if_des_type = 0x55;

	// Gửi handshake
	uart_boot_data_cmd_t frame{};
	frame.boot_cmd.cmd = UART_BOOT_CMD_HANDSHAKE_REQ;
	memcpy(frame.data, &fw_boot_cmd, sizeof(fw_boot_cmd));
	frame.len = sizeof(fw_boot_cmd);
	sendFrameWithACK((uint8_t *)&frame, sizeof(uart_boot_cmd_t) + frame.len);

	// const char *filename = "snake.bin";
	const char *filename ="idle.bin";
	// const char *filename ="ak.bin";
	// Gửi update request với metadata
	firmware_header_t meta = {};
	calculateFirmwareChecksum(filename, meta);
	meta.psk = FIRMWARE_PSK;

	frame.boot_cmd.cmd = UART_BOOT_CMD_UPDATE_REQ;
	memcpy(frame.data, &meta, sizeof(meta));
	frame.len = sizeof(meta);
	sendFrameWithACK((uint8_t *)&frame, sizeof(uart_boot_cmd_t) + frame.len);

	// FILE *fw = fopen("ak.bin", "rb");
	FILE *fw = fopen(filename, "rb");
	// FILE *fw = fopen("start.bin", "rb");
	if (!fw)
	{
		printf("Cannot open firmware file\n");
		return -1;
	}

	uint8_t buffer[CHUNK_SIZE];
	size_t bytes_read;
	size_t total_size = meta.bin_len;
	size_t total_chunks = (total_size + CHUNK_SIZE - 1) / CHUNK_SIZE;
	size_t chunk_index = 0;

	uint32_t offset = 0;

	while (offset < meta.bin_len)
	{
		uint32_t remaining = meta.bin_len - offset;
		uint32_t chunk = (remaining > CHUNK_SIZE) ? CHUNK_SIZE : remaining;

		// đọc đúng chunk
		size_t bytes_read = fread(buffer, 1, chunk, fw);
		if (bytes_read != chunk)
		{
			std::cout << "READ ERROR: expected " << chunk << ", got " << bytes_read << endl;
			return -1;
		}

		uart_boot_data_cmd_t frame{};
		frame.boot_cmd.cmd = UART_BOOT_CMD_TRANSFER_FW_REQ;
		frame.boot_cmd.subcmd = 0;
		frame.len = chunk; // CHỈ GỬI bytes thực sự

		memcpy(frame.data, buffer, chunk);

		uint16_t payload_size = 3 + frame.len; // cmd(1) + subcmd(1) + len(1) + data

		if (!sendFrameRetry((uint8_t *)&frame, payload_size))
		{
			std::cout << "UPLOAD FAILED!" << endl;
			return -1;
		}

		offset += chunk;
		chunk_index++;

		std::cout << "chunk_index: " << chunk_index << endl;
		// printProgress(chunk_index, total_chunks);
	}

	std::cout << "SEND ALL DATA!" << endl;

	std::cout << "FINAL CHECKSUM SEND!\n";
	state = SEND_FCS_STATE;
	frame.boot_cmd.cmd = SYS_BOOT_CMD_UPDATE_REQ; // frame để MCU chạy handler
	frame.boot_cmd.subcmd = 0;					  // hoặc 1, tùy MCU xử lý
	frame.len = 1;								  // để khác 0
	frame.data[0] = 0x00;
	if (!sendFrameWithACK((uint8_t *)&frame, sizeof(uart_boot_cmd_t) + frame.len))
	{
		std::cout << "FWU FAIL: WRONG FCS\n";
	}
	else
	{
		std::cout << "UART FWU DONE!\n";
	}

	// waitForAck(fd);
	close_uart(fd);
		// std::cout << "ENDDDD!\n";

	return 0;
}
