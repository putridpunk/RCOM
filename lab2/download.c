/**
 * RCOM 2025/26 - Lab 2, Part 1
 * Simple FTP download client (RFC 959) with URL parsing (RFC 1738).
 *
 * Usage:
 *   ./download ftp://[<user>:<password>@]<host>[:<port>]/<url-path>
 *
 * The client performs the single use case required by the lab:
 * connect to the server, login, enter passive mode, retrieve one file and
 * save it in the current working directory.
 *
 * This code is based on the socket examples provided with the assignment
 * (clientTCP.c and getip.c).  Name resolution uses getaddrinfo(), as
 * recommended by Beej's Guide to Network Programming.
 */

#define _POSIX_C_SOURCE 200809L

#include <arpa/inet.h>
#include <ctype.h>
#include <errno.h>
#include <netdb.h>
#include <netinet/in.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#define DEFAULT_FTP_PORT 21
#define URL_MAX_LEN 4096
#define HOST_MAX_LEN 256
#define USER_MAX_LEN 256
#define PASS_MAX_LEN 256
#define PATH_MAX_LEN 4096
#define LINE_MAX_LEN 2048
#define RESPONSE_MAX_LEN 8192
#define DATA_BUFFER_SIZE 8192

typedef struct {
    char user[USER_MAX_LEN];
    char password[PASS_MAX_LEN];
    char host[HOST_MAX_LEN];
    int port;
    char path[PATH_MAX_LEN];
} ftp_url_t;

static void set_error(char *err, size_t err_len, const char *fmt, ...) {
    va_list ap;

    if (err == NULL || err_len == 0)
        return;

    va_start(ap, fmt);
    vsnprintf(err, err_len, fmt, ap);
    va_end(ap);
}

static void usage(const char *program) {
    fprintf(stderr,
            "Usage: %s ftp://[<user>:<password>@]<host>[:<port>]/<url-path>\n"
            "\n"
            "Examples:\n"
            "  %s ftp://ftp.netlab.fe.up.pt/pub/teste.txt\n"
            "  %s ftp://rcom:rcom@ftp.netlab.fe.up.pt/pipe.txt\n"
            "  %s ftp://demo:password@test.rebex.net/readme.txt\n",
            program, program, program, program);
}

static int copy_string(char *destination, size_t destination_size,
                       const char *source) {
    size_t length = strlen(source);

    if (length >= destination_size)
        return -1;

    memcpy(destination, source, length + 1);
    return 0;
}

static int hex_value(int c) {    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    return -1;
}

/* Decode %XX escapes in place (RFC 1738).  '+' is kept as a literal '+'. */
static void url_decode(char *text) {
    char *read = text;
    char *write = text;

    while (*read != '\0') {
        if (*read == '%' && isxdigit((unsigned char)read[1]) &&
            isxdigit((unsigned char)read[2])) {
            int high = hex_value((unsigned char)read[1]);
            int low = hex_value((unsigned char)read[2]);
            *write++ = (char)((high << 4) | low);
            read += 3;
        } else {
            *write++ = *read++;
        }
    }
    *write = '\0';
}

/**
 * Parse ftp://[user[:password]@]host[:port]/url-path into its components.
 * The path is stored without the first '/' that separates it from the host.
 */
static int parse_url(const char *url, ftp_url_t *parsed, char *err,
                     size_t err_len) {
    char authority[HOST_MAX_LEN + USER_MAX_LEN + PASS_MAX_LEN + 32];
    char *host_port;
    char *at;
    char *colon;
    const char *authority_end;
    const char *path;
    size_t authority_len;
    int port = DEFAULT_FTP_PORT;

    memset(parsed, 0, sizeof(*parsed));
    parsed->port = DEFAULT_FTP_PORT;

    if (strncasecmp(url, "ftp://", 6) != 0) {
        set_error(err, err_len, "URL must start with ftp://");
        return -1;
    }

    authority_end = strchr(url + 6, '/');
    authority_len = authority_end ? (size_t)(authority_end - (url + 6))
                                  : strlen(url + 6);

    if (authority_len == 0) {
        set_error(err, err_len, "URL has no host");
        return -1;
    }
    if (authority_len >= sizeof(authority)) {
        set_error(err, err_len, "URL authority is too long");
        return -1;
    }

    memcpy(authority, url + 6, authority_len);
    authority[authority_len] = '\0';

    host_port = authority;
    at = strrchr(authority, '@');
    if (at != NULL) {
        *at = '\0';
        host_port = at + 1;

        colon = strchr(authority, ':');
        if (colon != NULL) {
            *colon = '\0';
            if (copy_string(parsed->user, sizeof(parsed->user), authority) < 0 ||
                copy_string(parsed->password, sizeof(parsed->password),
                            colon + 1) < 0) {
                set_error(err, err_len, "user or password is too long");
                return -1;
            }
        } else {
            if (copy_string(parsed->user, sizeof(parsed->user), authority) <
                0) {
                set_error(err, err_len, "user name is too long");
                return -1;
            }
        }
    }

    if (host_port[0] == '[') {
        char *closing = strchr(host_port, ']');

        if (closing == NULL) {
            set_error(err, err_len, "invalid IPv6 address in URL");
            return -1;
        }
        *closing = '\0';
        if (copy_string(parsed->host, sizeof(parsed->host), host_port + 1) <
            0) {
            set_error(err, err_len, "host name is too long");
            return -1;
        }

        if (closing[1] == ':') {
            port = atoi(closing + 2);
        } else if (closing[1] != '\0') {
            set_error(err, err_len, "invalid characters after IPv6 address");
            return -1;
        }
    } else {
        colon = strrchr(host_port, ':');
        if (colon != NULL) {
            *colon = '\0';
            port = atoi(colon + 1);
        }
        if (copy_string(parsed->host, sizeof(parsed->host), host_port) < 0) {
            set_error(err, err_len, "host name is too long");
            return -1;
        }
    }

    if (parsed->host[0] == '\0') {
        set_error(err, err_len, "URL has no host");
        return -1;
    }
    if (port <= 0 || port > 65535) {
        set_error(err, err_len, "invalid FTP port");
        return -1;
    }
    parsed->port = port;

    if (authority_end == NULL) {
        set_error(err, err_len, "URL has no remote path");
        return -1;
    }

    path = authority_end + 1;
    if (*path == '\0') {
        set_error(err, err_len, "URL has no remote path");
        return -1;
    }
    if (copy_string(parsed->path, sizeof(parsed->path), path) < 0) {
        set_error(err, err_len, "remote path is too long");
        return -1;
    }

    if (parsed->user[0] == '\0') {
        copy_string(parsed->user, sizeof(parsed->user), "anonymous");
        copy_string(parsed->password, sizeof(parsed->password),
                    "anonymous@");
    } else {
        url_decode(parsed->user);
        url_decode(parsed->password);
    }
    url_decode(parsed->path);

    return 0;
}

/**
 * Create a TCP connection to host:port.  getaddrinfo() supports both IPv4
 * and IPv6 and replaces the gethostbyname() used in getip.c.
 */
static int tcp_connect(const char *host, int port, char *err,
                       size_t err_len) {
    struct addrinfo hints;
    struct addrinfo *result = NULL;
    struct addrinfo *entry;
    char port_string[16];
    int sockfd = -1;
    int last_errno = 0;
    int gai_status;

    snprintf(port_string, sizeof(port_string), "%d", port);

    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    gai_status = getaddrinfo(host, port_string, &hints, &result);
    if (gai_status != 0) {
        set_error(err, err_len, "cannot resolve %s: %s", host,
                  gai_strerror(gai_status));
        return -1;
    }

    for (entry = result; entry != NULL; entry = entry->ai_next) {
        char ip[INET6_ADDRSTRLEN] = "unknown";
        void *address = NULL;

        if (entry->ai_family == AF_INET) {
            address = &((struct sockaddr_in *)entry->ai_addr)->sin_addr;
        } else if (entry->ai_family == AF_INET6) {
            address = &((struct sockaddr_in6 *)entry->ai_addr)->sin6_addr;
        }
        if (address != NULL)
            inet_ntop(entry->ai_family, address, ip, sizeof(ip));

        sockfd = socket(entry->ai_family, entry->ai_socktype,
                        entry->ai_protocol);
        if (sockfd < 0) {
            last_errno = errno;
            continue;
        }

        if (connect(sockfd, entry->ai_addr, entry->ai_addrlen) == 0) {
            printf("Connected to %s (%s) port %d\n", host, ip, port);
            break;
        }

        last_errno = errno;
        close(sockfd);
        sockfd = -1;
    }

    freeaddrinfo(result);

    if (sockfd < 0) {
        set_error(err, err_len, "cannot connect to %s:%d: %s", host, port,
                  last_errno ? strerror(last_errno) : "no usable address");
        return -1;
    }

    return sockfd;
}

static int send_all(int sockfd, const char *buffer, size_t length) {
    size_t sent = 0;

    while (sent < length) {
#ifdef MSG_NOSIGNAL
        int flags = MSG_NOSIGNAL;
#else
        int flags = 0;
#endif
        ssize_t n = send(sockfd, buffer + sent, length - sent, flags);

        if (n < 0) {
            if (errno == EINTR)
                continue;
            return -1;
        }
        if (n == 0)
            return -1;
        sent += (size_t)n;
    }

    return 0;
}

/* Send one FTP command terminated by CRLF.  Password commands are masked. */
static int ftp_command(int sockfd, bool secret, const char *fmt, ...) {
    char command[URL_MAX_LEN];
    va_list ap;
    int length;

    va_start(ap, fmt);
    length = vsnprintf(command, sizeof(command), fmt, ap);
    va_end(ap);

    if (length < 0 || (size_t)length >= sizeof(command)) {
        fprintf(stderr, "error: FTP command is too long\n");
        return -1;
    }

    printf("C: %s\n", secret ? "PASS ****" : command);
    fflush(stdout);

    if (send_all(sockfd, command, strlen(command)) < 0 ||
        send_all(sockfd, "\r\n", 2) < 0) {
        fprintf(stderr, "error: cannot send FTP command: %s\n",
                strerror(errno));
        return -1;
    }

    return 0;
}

/* Read a CRLF-terminated line, one byte at a time (control connection only). */
static int read_line(int sockfd, char *buffer, size_t size) {
    size_t used = 0;

    if (size == 0)
        return -1;

    for (;;) {
        char c;
        ssize_t n = recv(sockfd, &c, 1, 0);

        if (n == 0) {
            buffer[used] = '\0';
            return used == 0 ? 0 : (int)used;
        }
        if (n < 0) {
            if (errno == EINTR)
                continue;
            return -1;
        }
        if (c == '\n')
            break;
        if (c != '\r' && used + 1 < size)
            buffer[used++] = c;
    }

    buffer[used] = '\0';
    return (int)used;
}

static void append_line(char *text, size_t size, const char *line) {
    size_t used = strlen(text);

    if (used + 1 >= size)
        return;

    snprintf(text + used, size - used, "%s%s", used > 0 ? "\n" : "", line);
}

/**
 * Read one complete FTP response.  Handles multi-line replies such as
 * "220-..." ... "220 " and stores the whole reply in text.
 */
static int ftp_read_response(int sockfd, int *code, char *text,
                             size_t text_len) {
    char line[LINE_MAX_LEN];
    char code_string[4] = {0};
    int first_code = -1;
    bool multiline = false;

    if (text_len == 0)
        return -1;
    text[0] = '\0';
    *code = -1;

    for (;;) {
        int length = read_line(sockfd, line, sizeof(line));

        if (length <= 0)
            return -1;

        if (first_code < 0) {
            if (length < 3 || !isdigit((unsigned char)line[0]) ||
                !isdigit((unsigned char)line[1]) ||
                !isdigit((unsigned char)line[2])) {
                return -1;
            }

            first_code = atoi(line);
            memcpy(code_string, line, 3);
            code_string[3] = '\0';
            append_line(text, text_len, line);

            if (length >= 4 && line[3] == '-') {
                multiline = true;
                continue;
            }
        } else {
            append_line(text, text_len, line);

            if (strncmp(line, code_string, 3) == 0 &&
                (line[3] == ' ' || line[3] == '\0')) {
                break;
            }
            if (multiline)
                continue;
        }

        break;
    }

    printf("S: %s\n", text);
    fflush(stdout);
    *code = first_code;
    return 0;
}

/**
 * Read a response and require one of two codes (pass 0 for no second code).
 */
static int ftp_expect(int sockfd, const char *phase, int expected,
                      int alternative, char *text, size_t text_len) {
    int code;

    if (ftp_read_response(sockfd, &code, text, text_len) < 0) {
        fprintf(stderr, "error: %s: no response from server\n", phase);
        return -1;
    }

    if (code != expected && (alternative == 0 || code != alternative)) {
        fprintf(stderr, "error: %s failed (server replied %d)\n", phase,
                code);
        return -1;
    }

    return code;
}

/* Parse the "227 Entering Passive Mode (h1,h2,h3,h4,p1,p2)" reply. */
static int parse_pasv(const char *text, char *ip, size_t ip_len, int *port) {
    int h1, h2, h3, h4, p1, p2;
    const char *start = strchr(text, '(');
    int matched;

    if (start != NULL)
        matched = sscanf(start, "(%d,%d,%d,%d,%d,%d)", &h1, &h2, &h3, &h4,
                         &p1, &p2);
    else
        matched = sscanf(text, "%d,%d,%d,%d,%d,%d", &h1, &h2, &h3, &h4, &p1,
                         &p2);

    if (matched != 6 || h1 < 0 || h1 > 255 || h2 < 0 || h2 > 255 ||
        h3 < 0 || h3 > 255 || h4 < 0 || h4 > 255 || p1 < 0 || p1 > 255 ||
        p2 < 0 || p2 > 255) {
        return -1;
    }

    if (snprintf(ip, ip_len, "%d.%d.%d.%d", h1, h2, h3, h4) >= (int)ip_len)
        return -1;

    *port = p1 * 256 + p2;
    return (*port > 0) ? 0 : -1;
}

/* Split "dir/file" into "dir" and "file". */
static int split_path(const char *path, char *directory, size_t dir_len,
                      char *filename, size_t file_len) {
    const char *slash = strrchr(path, '/');

    if (slash == NULL) {
        directory[0] = '\0';
        if (strlen(path) >= file_len)
            return -1;
        snprintf(filename, file_len, "%s", path);
    } else {
        size_t dir_size = (size_t)(slash - path);

        if (dir_size >= dir_len || strlen(slash + 1) >= file_len)
            return -1;

        memcpy(directory, path, dir_size);
        directory[dir_size] = '\0';
        snprintf(filename, file_len, "%s", slash + 1);
    }

    if (filename[0] == '\0' || strcmp(filename, ".") == 0 ||
        strcmp(filename, "..") == 0) {
        return -1;
    }

    return 0;
}

static int ftp_download(const ftp_url_t *url) {
    int control = -1;
    int data = -1;
    FILE *output = NULL;
    char error[512];
    char response[RESPONSE_MAX_LEN];
    char directory[PATH_MAX_LEN];
    char filename[PATH_MAX_LEN];
    char remote_file[PATH_MAX_LEN];
    char local_file[PATH_MAX_LEN];
    char pasv_ip[64];
    int pasv_port = 0;
    int code;
    int result = -1;
    bool directory_changed = false;
    bool file_created = false;
    unsigned long long total_bytes = 0;
    char buffer[DATA_BUFFER_SIZE];

    local_file[0] = '\0';

    if (split_path(url->path, directory, sizeof(directory), filename,
                   sizeof(filename)) < 0) {
        fprintf(stderr, "error: invalid remote path '%s'\n", url->path);
        return -1;
    }
    snprintf(local_file, sizeof(local_file), "%s", filename);

    printf("\n--- Control connection ---\n");
    control = tcp_connect(url->host, url->port, error, sizeof(error));
    if (control < 0) {
        fprintf(stderr, "error: %s\n", error);
        goto cleanup;
    }

    if (ftp_expect(control, "server greeting", 220, 0, response,
                   sizeof(response)) < 0) {
        goto cleanup;
    }

    printf("\n--- Login ---\n");
    if (ftp_command(control, false, "USER %s", url->user) < 0)
        goto cleanup;

    if (ftp_read_response(control, &code, response, sizeof(response)) < 0) {
        fprintf(stderr, "error: login: no response from server\n");
        goto cleanup;
    }

    if (code == 331) {
        if (ftp_command(control, true, "PASS %s", url->password) < 0)
            goto cleanup;
        if (ftp_expect(control, "password authentication", 230, 0, response,
                       sizeof(response)) < 0) {
            goto cleanup;
        }
    } else if (code != 230) {
        fprintf(stderr, "error: login failed (server replied %d)\n", code);
        goto cleanup;
    }

    printf("\n--- Transfer type ---\n");
    if (ftp_command(control, false, "TYPE I") < 0)
        goto cleanup;
    if (ftp_expect(control, "TYPE I", 200, 0, response, sizeof(response)) < 0)
        goto cleanup;

    /*
     * Change to the directory that contains the file, following the example
     * in the lab guide (CWD debian; RETR README).  If the relative CWD fails,
     * try an absolute path.  If both fail, RETR the whole path directly.
     */
    if (directory[0] != '\0') {
        printf("\n--- Remote directory ---\n");
        if (ftp_command(control, false, "CWD %s", directory) < 0)
            goto cleanup;

        if (ftp_read_response(control, &code, response, sizeof(response)) <
            0) {
            fprintf(stderr, "error: CWD: no response from server\n");
            goto cleanup;
        }

        if (code == 250) {
            directory_changed = true;
        } else if (directory[0] != '/') {
            if (ftp_command(control, false, "CWD /%s", directory) < 0)
                goto cleanup;
            if (ftp_read_response(control, &code, response,
                                  sizeof(response)) < 0) {
                fprintf(stderr, "error: CWD: no response from server\n");
                goto cleanup;
            }
            directory_changed = (code == 250);
        }

        printf("%s\n", directory_changed
                           ? "Changed to the remote directory."
                           : "Could not change directory; using full path.");
    }

    snprintf(remote_file, sizeof(remote_file), "%s",
             directory_changed ? filename : url->path);

    printf("\n--- Local file ---\n");
    output = fopen(local_file, "wb");
    if (output == NULL) {
        fprintf(stderr, "error: cannot create '%s': %s\n", local_file,
                strerror(errno));
        goto cleanup;
    }
    file_created = true;
    printf("Saving to %s\n", local_file);

    printf("\n--- Passive mode ---\n");
    if (ftp_command(control, false, "PASV") < 0)
        goto cleanup;
    if (ftp_expect(control, "PASV", 227, 0, response, sizeof(response)) < 0)
        goto cleanup;
    if (parse_pasv(response, pasv_ip, sizeof(pasv_ip), &pasv_port) < 0) {
        fprintf(stderr, "error: cannot parse PASV response\n");
        goto cleanup;
    }

    printf("Opening data connection to %s port %d\n", pasv_ip, pasv_port);
    data = tcp_connect(pasv_ip, pasv_port, error, sizeof(error));
    if (data < 0) {
        fprintf(stderr, "error: data connection: %s\n", error);
        goto cleanup;
    }

    printf("\n--- Retrieve ---\n");
    if (ftp_command(control, false, "RETR %s", remote_file) < 0)
        goto cleanup;
    if (ftp_expect(control, "RETR", 150, 125, response, sizeof(response)) < 0)
        goto cleanup;

    for (;;) {
        ssize_t n = recv(data, buffer, sizeof(buffer), 0);

        if (n == 0)
            break;
        if (n < 0) {
            if (errno == EINTR)
                continue;
            fprintf(stderr, "error: reading data connection: %s\n",
                    strerror(errno));
            goto cleanup;
        }
        if (fwrite(buffer, 1, (size_t)n, output) != (size_t)n) {
            fprintf(stderr, "error: writing '%s': %s\n", local_file,
                    strerror(errno));
            goto cleanup;
        }
        total_bytes += (unsigned long long)n;
    }

    if (fclose(output) != 0) {
        output = NULL;
        fprintf(stderr, "error: closing '%s': %s\n", local_file,
                strerror(errno));
        goto cleanup;
    }
    output = NULL;

    close(data);
    data = -1;

    printf("Received %llu bytes.\n", total_bytes);

    if (ftp_expect(control, "transfer completion", 226, 250, response,
                   sizeof(response)) < 0) {
        goto cleanup;
    }

    printf("\n--- Logout ---\n");
    if (ftp_command(control, false, "QUIT") == 0) {
        /* The reply to QUIT is informative only. */
        ftp_read_response(control, &code, response, sizeof(response));
    }

    printf("\nDownload complete: %s (%llu bytes)\n", local_file, total_bytes);
    result = 0;

cleanup:
    if (output != NULL)
        fclose(output);
    if (data >= 0)
        close(data);
    if (control >= 0)
        close(control);
    if (result != 0 && file_created)
        remove(local_file);

    return result;
}

int main(int argc, char **argv) {
    ftp_url_t url;
    char error[512];

    if (argc != 2) {
        usage(argv[0]);
        return EXIT_FAILURE;
    }

    if (parse_url(argv[1], &url, error, sizeof(error)) < 0) {
        fprintf(stderr, "error: %s\n", error);
        return EXIT_FAILURE;
    }

    printf("FTP download\n");
    printf("  Host: %s\n", url.host);
    printf("  Port: %d\n", url.port);
    printf("  User: %s\n", url.user);
    printf("  Path: %s\n", url.path);

    return (ftp_download(&url) == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
