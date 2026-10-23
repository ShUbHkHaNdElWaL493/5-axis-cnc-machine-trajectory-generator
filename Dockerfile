FROM ubuntu:24.04 AS em-trajectory-build

SHELL ["/bin/bash", "-c"]

RUN apt update && apt install -y \
    build-essential \
    cmake \
    python3-matplotlib \
    python3-pandas \
    python3-plotly \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /em

COPY . .

RUN cmake -B build -DCMAKE_BUILD_TYPE=Release && \
    cmake --build build

FROM em-trajectory-build AS em-trajectory

CMD [ "bash" ]