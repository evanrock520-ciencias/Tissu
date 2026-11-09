FROM python:3.12-slim-bookworm

COPY --from=ghcr.io/astral-sh/uv:0.13.7 /uv /uvx /usr/local/bin/

RUN apt-get update && apt-get install -y \
    build-essential \
    git \
    ninja-build \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

RUN uv pip install --system --no-cache cmake==4.3.0

COPY . .

ENV CMAKE_BUILD_PARALLEL_LEVEL=4

RUN uv pip install --system --no-cache .

CMD ["python"]
