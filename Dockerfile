FROM intermission/reloc-ai-interview-base:ubuntu22.04-v1

WORKDIR /project
COPY . /project
RUN cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build -j2

ENTRYPOINT ["/project/build/reloc_benchmark"]
