FROM ubuntu:24.04 AS em-trajectory-build

SHELL ["/bin/bash", "-c"]

RUN apt update && apt install -y \
    build-essential \
    cmake \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /em

COPY ./include ./include
COPY ./src ./src
COPY ./CMakeLists.txt ./CMakeLists.txt

RUN cmake -B build -DCMAKE_BUILD_TYPE=Release && \
    cmake --build build

FROM ubuntu:24.04 AS em-trajectory

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-matplotlib \
    python3-pandas \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /em

COPY ./test ./test
COPY --from=em-trajectory-build /em/em-trajectory-generator ./em-trajectory-generator
COPY ./em-trajectory-visualizer.py ./em-trajectory-visualizer.py

CMD [ "bash" ]