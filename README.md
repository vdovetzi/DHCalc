# DHCalc

[![C++20](https://img.shields.io/badge/C%2B%2B-20-00599C?logo=c%2B%2B&logoColor=white)](https://en.cppreference.com/w/cpp/20)
[![CMake](https://img.shields.io/badge/CMake-3.20%2B-064F8C?logo=cmake&logoColor=white)](https://cmake.org/)
[![Eigen](https://img.shields.io/badge/Eigen-3.3%2B-blue)](https://eigen.tuxfamily.org/)
[![License: MIT](https://img.shields.io/badge/license-MIT-yellow.svg)](LICENSE)

DHCalc is a C++ command-line tool for Denavit-Hartenberg forward kinematics.

## Input File Format

A TSV (tab-separated values) is the chosen file format, because a DH table is already tabular data and TSV keeps the input easy to read, easy to generate from spreadsheets or scripts, and easy to parse in a lightweight CLI application.

### Supported Columns

| Column | Required | Alternatives |
|--------|----------|--------------|
| `r` | yes | — |
| `d` | yes | — |
| `θ` | yes, exactly one | `θ_rad` · `θ_deg` (by default) |
| `α` | yes, exactly one | `α_rad` · `α_deg` (by default) |
| `joint_name` | no | — |

Notes:

- Blank lines are ignored.
- Lines starting with `#` are treated as comments.
- Column order can vary.
- The parser expects simple tab-separated values without quoting.

### Example Input
See in [examples/stanford_arm.dh.tsv](examples/stanford_arm.dh.tsv)

At configure time CMake also generates `examples/empty.dh.tsv`, a header-only table with the canonical column order and zero data rows.

## Build

Requirements:

- CMake 3.20 or newer
- A C++20 compiler
- [Eigen 3.3+](https://eigen.tuxfamily.org/) installed on the system
- [GoogleTest](https://github.com/google/googletest) installed on the system (for tests)

Build commands:
```bash
mkdir build && cd build
cmake ..
make && cd ..
# or
cmake -B build
cmake --build build
```

## Run

Basic usage:
```bash
./build/dhcalc <path-to-dh-table.tsv>
```

Optional precision control:

``` bash
./build/dhcalc examples/stanford_arm.dh.tsv --precision 4
```

## Test
```bash
ctest --test-dir build --output-on-failure
```

| Binary | Directory | Purpose |
|--------|-----------|---------|
| `dhcalc_parser_tests` | `tests/parser/` | TSV parsing, header validation, error handling |
| `dhcalc_fk_tests` | `tests/fk/` | Joint transforms, chain multiplication, formatting |
| `dhcalc_stress_tests` | `tests/stress/` | Large generated tables in `/tmp/`, timing |