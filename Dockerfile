FROM python:3.12-slim-bookworm

COPY --from=ghcr.io/astral-sh/uv:0.13.7 /uv /uvx /usr/local/bin/

RUN apt-get update && apt-get install -y \
    build-essential \
    git \
    ninja-build \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

RUN uv pip install --system --no-cache cmake==4.3.0 jinja2

COPY . .

ENV CMAKE_BUILD_PARALLEL_LEVEL=4

RUN cmake --preset headless-release -DBUILD_TESTING=OFF \
    && cmake --build --preset headless-release

RUN uv pip install --system --no-cache .

CMD ["python"]
