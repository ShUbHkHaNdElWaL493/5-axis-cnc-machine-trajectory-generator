FROM ubuntu:24.04 AS em-planner-build

SHELL ["/bin/bash", "-c"]

RUN apt update && apt install -y \
    build-essential \
    cmake \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /em

COPY . .

RUN cmake -B build -DCMAKE_BUILD_TYPE=Release && \
    cmake --build build

FROM em-planner-build AS em-planner

CMD [ "bash" ]