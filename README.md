# RCOM — Computer Networks

Course project for **Computer Networks** in the 3rd year of **L.EIC** at FEUP.

The aim is to implement reliable file transfer over a serial link using a
provided serial-port API: the **link layer** (framing, error control,
retransmissions) and a simple **application protocol** (control and data
packets) built on top of it.

> The code uses `termios`, `alarm(2)`/`SIGALRM` and `socat`, so it only runs on
> Linux, macOS or WSL — not on native Windows.

## Repository structure

```
.
├── README.md
└── lab1/
    ├── src/            # implementation (link layer + application)
    │   ├── main.c             # argument parsing (do not change)
    │   ├── link_layer.c/.h    # link protocol: SET/UA, I, RR/REJ, DISC
    │   ├── frame.c/.h         # pure framing, BCC and byte-stuffing helpers
    │   ├── application_layer.c/.h  # START/DATA/END packets and TLVs
    │   ├── serial_port.c/.h   # serial-port API (do not change)
    │   └── alarm_sigaction.c/.h    # timeouts via alarm(2)
    ├── cable/cable.c   # virtual cable (do not change)
    ├── tests/selftest.c # tests for the framing helpers
    ├── Makefile        # build and run targets (do not change)
    ├── penguin.gif     # sample file
    └── README.txt      # original lab assignment
```

## Implemented protocol

### Link layer (`src/link_layer.c`)

HDLC-style framing with **stop-and-wait** and alternating sequence numbers
(0/1):

```
supervisory: F | A | C | BCC1 | F
information: F | A | C | BCC1 | data | BCC2 | F
```

- `F = 0x7E`, `A` = address (`0x03` command, `0x01` reply), and
  `BCC1 = A ^ C` (even parity).
- `BCC2` is the even parity of all data bytes.
- **Byte stuffing** applies to everything after `BCC1`:
  `0x7E → 0x7D 0x5E` and `0x7D → 0x7D 0x5D`.
- Connection setup uses **SET/UA** and teardown uses **DISC/UA**, each with up
  to `nRetransmissions + 1` attempts and a timeout driven by `SIGALRM`.
- Data is acknowledged with **RR** and retransmission is requested with **REJ**;
  duplicate frames are detected through `Ns` and discarded.
- `BCC1`/`BCC2` errors, timeouts and retransmissions are counted and printed at
  the end of the transfer (statistics, throughput and efficiency).

### Application layer (`src/application_layer.c`)

Packets are `C | L2 | L1 | payload`, where the payload size is
`256 * L2 + L1`:

| Type | `C` | Contents |
| ---- | --- | -------- |
| START | `1` | metadata TLVs |
| DATA | `2` | `APP_DATA_CHUNK` file bytes (997 by default) |
| END | `3` | the same TLVs as START |

Supported TLVs:

| Type | Name | Format |
| ---- | ---- | ------ |
| `0` | file size | 4 bytes, big endian |
| `1` | file name | 0–255 bytes, no `\0` |

The receiver validates the metadata, writes data as it arrives and confirms
that the received size matches the announced one. `MAX_PAYLOAD_SIZE` is 1000
bytes; the data chunk size can be changed at compile time with
`-DAPP_DATA_CHUNK=...` (used for efficiency measurements).

## Building

```bash
cd lab1
make            # builds bin/main and bin/cable
```

## Running

[`socat`](https://linux.die.net/man/1/socat) is required for the virtual cable.

```bash
# 1. Virtual cable (requires root)
make run_cable          # or: sudo ./bin/cable

# 2. Receiver
make run_rx             # ./bin/main /tmp/ttyS11 9600 rx penguin-received.gif

# 3. Transmitter
make run_tx             # ./bin/main /tmp/ttyS10 9600 tx penguin.gif

# 4. Check integrity
make check_files        # diff -s penguin.gif penguin-received.gif
```

Running the executable directly:

```
./bin/main <port> <baudrate> <tx|rx> <file>
```

`nTries = 3` and `timeout = 4 s` are defined in `src/main.c`; the supported baud
rates are 1200, 1800, 2400, 4800, 9600, 19200, 38400, 57600 and 115200.

### Testing with errors

With the virtual cable running, enter commands in its terminal:

```
off              # disconnect the link
on               # reconnect
ber 1e-4         # inject noise (bit error rate)
prop 1000        # propagation delay in microseconds
log frames.txt   # log exchanged bytes
quit
```

The transfer should still produce a file identical to the original, even with
interrupted links or noise.

## Tests

The framing helpers have tests that are independent of the state machine:

```bash
cd lab1
gcc -Wall -Wextra -I src src/frame.c tests/selftest.c -o tests/selftest
./tests/selftest
```

## Notes

- `MAX_FILE_SIZE` is `0xFFFFFFFF` (4 GiB − 1) and file names are limited to
  255 characters.
- `main.c`, `link_layer.h`, `application_layer.h`, `serial_port.*`,
  `cable/cable.c` and the `Makefile` are provided with the assignment and must
  not be changed.
