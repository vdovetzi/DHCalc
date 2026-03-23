# Dockerfile
FROM ubuntu:22.04 AS builder

# Install build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    ninja-build \
    libeigen3-dev \
    libgtest-dev \
    git \
    ca-certificates \
    libgmp-dev \
    libmpfr-dev \
    && update-ca-certificates && rm -rf /var/lib/apt/lists/*

ENV GIT_SSL_CAINFO=/etc/ssl/certs/ca-certificates.crt

# Copy the source code into the container
WORKDIR /workspace
COPY . .

# Configure and build the project (including tests)
RUN cmake -B build -DCMAKE_BUILD_TYPE=Release -DDHCALC_BUILD_TESTS=ON -G Ninja && \
    cmake --build build --parallel

FROM ubuntu:22.04

RUN apt-get update && apt-get install -y --no-install-recommends \
    libstdc++6 \
    cmake \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /workspace/build /workspace/build
COPY --from=builder /workspace/examples /workspace/examples

WORKDIR /workspace/build