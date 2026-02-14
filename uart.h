#pragma once
#include <cstddef>

/**
Universal Asynchronous Receiver/Transmitter (UART)
    Serial (character) device: 1 byte at a time
    UART: standard "state machine" at the end of serial line
    BCM Chap 11 (not Chap 2): UART0
        device registers at offset 0x201000
        data register (DR) at offset 0x00
        FIFO buffer (optional, default size 1)
    configuration: speed, byte representation, FIFO
        terminal: 115200 baud (8 bits, no parity, 1 stop bit)
    output (transmit, TX)
        write to DR
        UART moves to FIFO
        FIFO entries sent via serial line
    input (receive, RX)
        reception from serial line into FIFO
        first element in FIFO is always available in DR
        read from DR
    flag register (FR) describes status of TX/RX buffers (full/empty)
        check before send/receive
    timing
        actual communication happens in parallel to UART processing
        receive: need to be fast enough
        transmit: paced according to line speed when checking TX state in FR
        how long does it take to transmit a byte to the console?
        sender speed vs. serial line speed vs. receiver speed
        optional: flow control
*/
class Uart {
public:
  static inline constexpr size_t CONSOLE = 0;

  static void configAndEnable();
  static void syncPrint(const char* cstring);
  static char syncRead();

  static bool tryPutc(unsigned char c);
  static bool tryGetc(unsigned char& ch);

  static void enableRxInterrupt();
  static void disableRxInterrupt();
  static void clearRxInterrupt();
  static bool hasRxInterrupt();

  static void enableTxInterrupt();
  static void disableTxInterrupt();
  static void clearTxInterrupt();
  static bool hasTxInterrupt();
};
