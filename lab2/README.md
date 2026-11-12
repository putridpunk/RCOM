# Lab 2 — FTP download application (Part 1)

Simple FTP client written in C for the RCOM lab assignment. It downloads one
file from an FTP server using the URL syntax defined in RFC 1738 and the FTP
protocol defined in RFC 959.

## Scope

This part covers the **first class** of Lab 2:

- FTP experiments and the client/server model;
- DNS resolution and TCP sockets;
- URL parsing;
- the single download use case (connect, login, passive mode, `RETR`, save).

Part 2 (network configuration experiments 1–6) is **not** implemented here;
it is performed on the laboratory bench.

## Files

| File | Description |
| ---- | ----------- |
| `download.c` | FTP client implementation (URL parsing, control connection, passive mode, download) |
| `Makefile` | Builds the `download` executable |
| `clientTCP.c` | Course-provided TCP example (unchanged) |
| `getip.c` | Course-provided name-resolution example (unchanged) |
| `docs/` | Local copies of the lab guide and Beej's Guide (ignored by Git) |

## Build

```bash
cd lab2
make
```

Requires a POSIX system (Linux, macOS or WSL) with a C compiler. On native
Windows the sockets API differs, so use WSL.

## Usage

```text
./download ftp://[<user>:<password>@]<host>[:<port>]/<url-path>
```

If no user is given, the client uses anonymous access:

- user: `anonymous`
- password: `anonymous@`

The file is saved in the current working directory with the last component of
the URL path as its name. An existing file with that name is overwritten.

Examples:

```bash
# NetLab anonymous access (only available inside the lab network)
./download ftp://ftp.netlab.fe.up.pt/pipe.txt
./download ftp://ftp.netlab.fe.up.pt/pub/teste.txt

# NetLab authenticated access
./download ftp://rcom:rcom@ftp.netlab.fe.up.pt/pipe.txt

# Public servers
./download ftp://demo:password@test.rebex.net/readme.txt
./download ftp://ftp.up.pt/pub/debian/README
```

## Architecture

The application follows a single use case: **connect, login, enter passive
mode, retrieve one file, save it locally**.

```text
URL ──► parse_url() ──► getaddrinfo() ──► control TCP connection (:21)
                                              │
                                              ├── 220 greeting
                                              ├── USER / PASS
                                              ├── TYPE I
                                              ├── CWD <dir> (optional)
                                              ├── PASV ──► 227 (ip, port)
                                              │              │
                                              │              └── data TCP connection
                                              ├── RETR <file> ──► 150/125
                                              │              │
                                              │              └── file bytes until EOF
                                              ├── 226 transfer complete
                                              └── QUIT ──► 221
```

### URL parsing

`parse_url()` splits `ftp://[user[:password]@]host[:port]/path` into its
components and decodes `%XX` escapes. IPv6 hosts can be written as `[::1]`.
The path is kept without the first `/` after the host, so
`ftp://host/pub/teste.txt` becomes `pub/teste.txt`.

### Control connection

The control connection is a normal TCP connection to port 21 (or the port in
the URL). FTP replies are read line by line, including multi-line replies
whose first line uses `NNN-` and whose last line uses `NNN `.

### Login

`USER` is always sent. If the server replies `331`, `PASS` follows. A reply of
`230` to `USER` means that no password is required. The password is masked in
the client log.

### Passive mode

`PASV` asks the server for the data address. The reply

```text
227 Entering Passive Mode (h1,h2,h3,h4,p1,p2)
```

is converted to `h1.h2.h3.h4` and port `256 * p1 + p2`. The client opens the
data TCP connection before sending `RETR`, as required in passive mode.

### Retrieval

If the URL contains a directory, the client first tries `CWD <dir>`, then
`CWD /<dir>` as a fallback, and finally `RETR <whole-path>` if neither
succeeds. This follows the guide's `CWD debian` / `RETR README` example and
also supports servers whose anonymous home directory is not `/`.

The data connection is read until EOF and written to the local file. The
client then reads the final `226` (or `250`) reply and sends `QUIT`.

## Error reporting

Failures identify the phase in which they occurred, for example:

- `URL must start with ftp://`
- `cannot resolve <host>: ...`
- `cannot connect to <host>:<port>: ...`
- `server greeting failed`
- `login failed`
- `PASV failed`
- `data connection: ...`
- `RETR failed`
- `transfer completion failed`

If a transfer fails after the local file was created, the partial file is
removed.

## Limitations

- Only passive mode is implemented (`PASV`, IPv4 data connection).
- Only binary mode (`TYPE I`) is used, which is correct for arbitrary files.
- Only one file is downloaded per invocation.
- No TLS/FTPS, restart, resume, or directory listing.
