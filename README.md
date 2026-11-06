# RCOM — Redes de Computadores

Trabalho da cadeira de **Redes de Computadores** do 3.º ano da **L.EIC** (FEUP).

O objetivo é implementar uma transferência de ficheiros fiável sobre uma ligação
série, usando uma API de porta série fornecida: a **camada de ligação** (framing,
controlo de erros, retransmissões) e um **protocolo de aplicação** simples
(pacotes de controlo e de dados) por cima dela.

> O código usa `termios`, `alarm(2)`/`SIGALRM` e `socat`, por isso corre apenas em
> Linux, macOS ou WSL — não em Windows nativo.

## Estrutura do repositório

```
.
├── README.md
└── lab1/
    ├── src/            # implementação (camada de ligação + aplicação)
    │   ├── main.c             # parsing dos argumentos (não alterar)
    │   ├── link_layer.c/.h    # protocolo de ligação: SET/UA, I, RR/REJ, DISC
    │   ├── frame.c/.h         # helpers puros de framing, BCC e byte stuffing
    │   ├── application_layer.c/.h  # pacotes START/DATA/END e TLVs
    │   ├── serial_port.c/.h   # API da porta série (não alterar)
    │   └── alarm_sigaction.c/.h    # timeouts via alarm(2)
    ├── cable/cable.c   # cabo virtual (não alterar)
    ├── tests/selftest.c # testes dos helpers de framing
    ├── Makefile        # build e targets de execução (não alterar)
    ├── penguin.gif     # ficheiro de exemplo
    └── README.txt      # enunciado original do laboratório
```

## Protocolo implementado

### Camada de ligação (`src/link_layer.c`)

Framing do tipo HDLC, com **stop-and-wait** e números de sequência alternados
(0/1):

```
supervisão: F | A | C | BCC1 | F
informação: F | A | C | BCC1 | dados | BCC2 | F
```

- `F = 0x7E`, `A` = endereço (`0x03` comando, `0x01` resposta),
  `BCC1 = A ^ C` (paridade par).
- `BCC2` é a paridade par de todos os bytes de dados.
- **Byte stuffing** em tudo o que vem depois do `BCC1`:
  `0x7E → 0x7D 0x5E` e `0x7D → 0x7D 0x5D`.
- Estabelecimento de ligação com **SET/UA**, terminação com **DISC/UA**,
  ambas com até `nRetransmissions + 1` tentativas e timeout por `SIGALRM`.
- Dados confirmados com **RR** e pedidos de reenvio com **REJ**; tramas
  duplicadas são detetadas pelo `Ns` e ignoradas.
- Erros de `BCC1`/`BCC2`, timeouts e reenvios são contabilizados e impressos no
  fim da transferência (estatísticas, throughput e eficiência).

### Camada de aplicação (`src/application_layer.c`)

Pacotes `C | L2 | L1 | payload`, em que o tamanho do payload é
`256 * L2 + L1`:

| Tipo | `C` | Conteúdo |
| ---- | --- | -------- |
| START | `1` | TLVs de metadados |
| DATA | `2` | `APP_DATA_CHUNK` bytes de ficheiro (997 por omissão) |
| END | `3` | os mesmos TLVs do START |

TLVs suportados:

| Tipo | Nome | Formato |
| ---- | ---- | ------- |
| `0` | tamanho do ficheiro | 4 bytes, big endian |
| `1` | nome do ficheiro | 0–255 bytes, sem `\0` |

O recetor valida os metadados, escreve os dados à medida que chegam e confirma
que o tamanho recebido corresponde ao anunciado. `MAX_PAYLOAD_SIZE` é 1000
bytes; o tamanho dos blocos de dados pode ser alterado em tempo de compilação
com `-DAPP_DATA_CHUNK=...` (usado para medir eficiência).

## Compilar

```bash
cd lab1
make            # constrói bin/main e bin/cable
```

## Correr

É necessário o [`socat`](https://linux.die.net/man/1/socat) para o cabo virtual.

```bash
# 1. Cabo virtual (precisa de root)
make run_cable          # ou: sudo ./bin/cable

# 2. Recetor
make run_rx             # ./bin/main /tmp/ttyS11 9600 rx penguin-received.gif

# 3. Transmissor
make run_tx             # ./bin/main /tmp/ttyS10 9600 tx penguin.gif

# 4. Verificar integridade
make check_files        # diff -s penguin.gif penguin-received.gif
```

Uso direto do executável:

```
./bin/main <porta> <baudrate> <tx|rx> <ficheiro>
```

`nTries = 3` e `timeout = 4 s` estão definidos em `src/main.c`; os baudrates
suportados são 1200, 1800, 2400, 4800, 9600, 19200, 38400, 57600 e 115200.

### Testar com erros

Com o cabo virtual a correr, introduza comandos no seu terminal:

```
off              # corta a ligação
on               # volta a ligar
ber 1e-4         # injeta ruído (bit error rate)
prop 1000        # atraso de propagação em microssegundos
log frames.txt   # registar bytes trocados
quit
```

A transferência deve continuar a produzir um ficheiro idêntico ao original,
mesmo com ligações interrompidas ou ruído.

## Testes

Os helpers de framing têm testes independentes da máquina de estados:

```bash
cd lab1
gcc -Wall -Wextra -I src src/frame.c tests/selftest.c -o tests/selftest
./tests/selftest
```

## Notas

- `MAX_FILE_SIZE` é `0xFFFFFFFF` (4 GiB − 1) e o nome do ficheiro está limitado
  a 255 caracteres.
- Os ficheiros `main.c`, `link_layer.h`, `application_layer.h`,
  `serial_port.*`, `cable/cable.c` e o `Makefile` são fornecidos no enunciado e
  não devem ser alterados.
