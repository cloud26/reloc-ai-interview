FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates cmake g++ make \
    libpcl-dev libeigen3-dev libyaml-cpp-dev libtbb-dev libomp-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /project
COPY . /project
RUN cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build -j2

ENTRYPOINT ["/project/build/reloc_benchmark"]
