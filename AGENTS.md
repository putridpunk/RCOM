# Agent instructions

## Do not delete or ignore `README.md`

`README.md` at the repository root is required project documentation and is
tracked by Git. AI coding agents must **not** (unless specifically asked to):

- delete, rename, truncate, or replace it with a stub;
- remove it from Git tracking (`git rm`) or add it to `.gitignore`;
- run destructive cleanup commands that could remove it
  (for example `git clean -fdx` without checking what would be removed).

When documentation needs to change, edit `README.md` in place and keep its
content accurate.

## Course-provided files

The following files come with the lab assignment and should not be modified:
`lab1/src/main.c`, `lab1/src/link_layer.h`, `lab1/src/application_layer.h`,
`lab1/src/serial_port.c`, `lab1/src/serial_port.h`, `lab1/cable/cable.c` and
`lab1/Makefile`.
