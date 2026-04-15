# =========================
# Stage 1: webui builder
# =========================
FROM nvidia/cuda:12.5.1-devel-ubuntu22.04 AS webui-builder

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=UTC \
    PIP_NO_CACHE_DIR=1 \
    CUDA_HOME=/usr/local/cuda \
    TORCH_CUDA_ARCH_LIST="8.6;8.9+PTX"

WORKDIR /build

ARG PIP_INDEX_URL
ARG PYTORCH_INDEX_URL=https://download.pytorch.org/whl/cu121
ARG LLM_REPO=https://github.com/oobabooga/text-generation-webui.git
ARG LLM_REF=v3.23
ARG GITHUB_RELEASES_PROXY_BASE

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 \
    python3-pip \
    python3-venv \
    python3-dev \
    build-essential \
    git \
    ca-certificates \
    curl \
    wget \
    && ln -snf /usr/share/zoneinfo/$TZ /etc/localtime \
    && echo $TZ > /etc/timezone \
    && rm -rf /var/lib/apt/lists/*

RUN if [ -n "$PIP_INDEX_URL" ]; then \
      mkdir -p /root/.config/pip && \
      printf '[global]\nindex-url = %s\n' "$PIP_INDEX_URL" > /root/.config/pip/pip.conf; \
    fi

# one system torch for this stage
RUN python3 -m pip install --upgrade pip && \
    python3 -m pip install \
      torch==2.4.0 \
      torchvision==0.19.0 \
      torchaudio==2.4.0 \
      --index-url "${PYTORCH_INDEX_URL}"

# webui venv uses system torch
RUN python3 -m venv /opt/venv-webui --system-site-packages
ENV PATH="/opt/venv-webui/bin:$PATH"

RUN git clone --depth 1 --branch "${LLM_REF}" "${LLM_REPO}" /opt/text-generation-webui || \
    (git clone "${LLM_REPO}" /opt/text-generation-webui && \
     cd /opt/text-generation-webui && git checkout "${LLM_REF}")

RUN cd /opt/text-generation-webui && \
    if [ -n "$GITHUB_RELEASES_PROXY_BASE" ]; then \
      sed -i "/github.com\\/oobabooga/s|https://github.com/|${GITHUB_RELEASES_PROXY_BASE}|g" requirements/portable/requirements.txt; \
    fi

# webui deps WITHOUT torch duplication
RUN /opt/venv-webui/bin/pip install --upgrade pip && \
    cd /opt/text-generation-webui && \
    /opt/venv-webui/bin/pip install -r requirements/portable/requirements.txt --upgrade && \
    /opt/venv-webui/bin/pip install --no-deps huggingface_hub==0.25.2 && \
    /opt/venv-webui/bin/pip install transformers==4.34.1 accelerate==0.22 exllamav2 packaging ninja wheel setuptools

# force JIT build of exllamav2 extension in builder
RUN /opt/venv-webui/bin/python -c "import exllamav2" && \
    cp /root/.cache/torch_extensions/py310_cu121/exllamav2_ext/exllamav2_ext.so \
       /opt/venv-webui/lib/python3.10/site-packages/exllamav2_ext.so

RUN mkdir -p /opt/text-generation-webui/user_data/models

# cleanup builder caches
RUN rm -rf /root/.cache/pip /root/.cache/torch_extensions /tmp/* && \
    find /opt/venv-webui -type d -name __pycache__ -prune -exec rm -rf {} + && \
    find /opt/venv-webui -type f \( -name '*.pyc' -o -name '*.pyo' -o -name '*.a' \) -delete && \
    rm -rf /opt/text-generation-webui/.git

# tts venv
RUN python3 -m venv /opt/venv-tts --system-site-packages

ARG TTS_REPO=https://github.com/sweetie-bot-project/TTS.git
ARG TTS_REF=dev

RUN curl -fsSL \
    "https://raw.githubusercontent.com/sweetie-bot-project/TTS/${TTS_REF}/requirements.txt" \
    -o /tmp/requirements-voice-upstream.txt && \
    grep -Ev '^(torch|torchaudio|torchvision)' /tmp/requirements-voice-upstream.txt \
      | grep -Ev '^\s*$|^\s*#' \
      > /tmp/requirements-voice.txt && \
    echo "git+${TTS_REPO}@${TTS_REF}" >> /tmp/requirements-voice.txt

RUN /opt/venv-tts/bin/pip install --upgrade pip && \
    /opt/venv-tts/bin/pip install --no-cache-dir -r /tmp/requirements-voice.txt

# =========================
# Stage 2: final runtime
# =========================
FROM nvidia/cuda:12.5.1-cudnn-runtime-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=UTC \
    PIP_NO_CACHE_DIR=1 \
    HOME=/root \
    COQUI_TOS_AGREED=1

WORKDIR /app

ARG PIP_INDEX_URL
ARG PYTORCH_INDEX_URL=https://download.pytorch.org/whl/cu121
ARG ARGOS_PACKAGE_INDEX

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 \
    python3-pip \
    python3-venv \
    ca-certificates \
    curl \
    wget \
    git \
    iproute2 \
    tzdata \
    espeak-ng \
    libespeak-ng1 \
    ffmpeg \
    && ln -snf /usr/share/zoneinfo/$TZ /etc/localtime \
    && echo $TZ > /etc/timezone \
    && rm -rf /var/lib/apt/lists/*

RUN if [ -n "$PIP_INDEX_URL" ]; then \
      mkdir -p /root/.config/pip && \
      printf '[global]\nindex-url = %s\n' "$PIP_INDEX_URL" > /root/.config/pip/pip.conf; \
    fi

ENV ARGOS_PACKAGE_INDEX=${ARGOS_PACKAGE_INDEX}

# one shared system torch for main/webui/tts
RUN python3 -m pip install --upgrade pip && \
    python3 -m pip install \
      torch==2.4.0 \
      torchvision==0.19.0 \
      torchaudio==2.4.0 \
      --index-url "${PYTORCH_INDEX_URL}"

# venvs share system torch
RUN python3 -m venv /opt/venv-main --system-site-packages
#    python3 -m venv /opt/venv-webui --system-site-packages && \
#    python3 -m venv /opt/venv-tts --system-site-packages

# requirements
COPY behavior/sweetie_bot_stt/scripts/requirements.txt /tmp/requirements-stt.txt
COPY behavior/sweetie_bot_translate/scripts/requirements.txt /tmp/requirements-translate.txt
COPY behavior/sweetie_bot_llm/scripts/requirements-classification.txt /tmp/requirements-classification.txt
COPY hardware/sweetie_bot_mic/src/sweetie_bot_mic/Silero_vad/requirements.txt /tmp/requirements-silero.txt

# main services
RUN /opt/venv-main/bin/pip install --upgrade pip && \
    /opt/venv-main/bin/pip install -r /tmp/requirements-stt.txt && \
    /opt/venv-main/bin/pip install -r /tmp/requirements-translate.txt && \
    /opt/venv-main/bin/pip install -r /tmp/requirements-classification.txt && \
    /opt/venv-main/bin/pip install -r /tmp/requirements-silero.txt

# copy prebuilt webui venv + repo
COPY --from=webui-builder /opt/venv-webui /opt/venv-webui
COPY --from=webui-builder /opt/text-generation-webui /opt/text-generation-webui
COPY --from=webui-builder /opt/venv-tts /opt/venv-tts

# install docker-systemctl-replacement as before
RUN wget -O /usr/bin/systemctl3.py \
    https://raw.githubusercontent.com/gdraheim/docker-systemctl-replacement/master/files/docker/systemctl3.py && \
    wget -O /usr/bin/journalctl3.py \
    https://raw.githubusercontent.com/gdraheim/docker-systemctl-replacement/master/files/docker/journalctl3.py && \
    chmod +x /usr/bin/systemctl3.py /usr/bin/journalctl3.py && \
    ln -sf /usr/bin/systemctl3.py /usr/bin/systemctl && \
    ln -sf /usr/bin/journalctl3.py /usr/bin/journalctl && \
    mkdir -p /run/systemd/system /etc/systemd/system/multi-user.target.wants

# app dirs
RUN mkdir -p \
    /app/sweetie_bot_stt \
    /app/sweetie_bot_translate \
    /app/sweetie_bot_voice \
    /app/sweetie_bot_llm \
    /app/sweetie_bot_classification \
    /app/sweetie_bot_silero

# app files
COPY behavior/sweetie_bot_stt/scripts/app.py /app/sweetie_bot_stt/app.py
COPY behavior/sweetie_bot_stt/scripts/config.py /app/sweetie_bot_stt/config.py

COPY behavior/sweetie_bot_llm/scripts/classificator_app.py /app/sweetie_bot_classification/app.py
COPY behavior/sweetie_bot_llm/scripts/download_classification_model.py /app/sweetie_bot_classification/download_classification_model.py
COPY docker/start_llm.sh /app/sweetie_bot_llm/start_llm.sh
COPY behavior/sweetie_bot_llm/scripts/config.py /app/sweetie_bot_classification/config.py

COPY docker/check_tts_model_and_start_voice_service.py /app/sweetie_bot_voice/check_tts_model_and_start_voice_service.py

COPY hardware/sweetie_bot_mic/src/sweetie_bot_mic/Silero_vad/silero_server.py /app/sweetie_bot_silero/silero_server.py

# service files
COPY docker/sweetie_bot_stt.service /etc/systemd/system/sweetie_bot_stt.service
COPY docker/sweetie_bot_translate.service /etc/systemd/system/sweetie_bot_translate.service
COPY docker/sweetie_bot_voice.service /etc/systemd/system/sweetie_bot_voice.service
COPY docker/sweetie_bot_llm.service /etc/systemd/system/sweetie_bot_llm.service
COPY docker/sweetie_bot_classification.service /etc/systemd/system/sweetie_bot_classification.service
COPY docker/sweetie_bot_silero.service /etc/systemd/system/sweetie_bot_silero.service
COPY docker/start-libretranslate.sh /usr/local/bin/start-libretranslate.sh

RUN chmod +x /usr/local/bin/start-libretranslate.sh && \
    chmod +x /app/sweetie_bot_classification/download_classification_model.py && \
    chmod +x /app/sweetie_bot_llm/start_llm.sh && \
    chmod +x /app/sweetie_bot_voice/check_tts_model_and_start_voice_service.py && \
    ln -sf /etc/systemd/system/sweetie_bot_stt.service /etc/systemd/system/multi-user.target.wants/sweetie_bot_stt.service && \
    ln -sf /etc/systemd/system/sweetie_bot_translate.service /etc/systemd/system/multi-user.target.wants/sweetie_bot_translate.service && \
    ln -sf /etc/systemd/system/sweetie_bot_voice.service /etc/systemd/system/multi-user.target.wants/sweetie_bot_voice.service && \
    ln -sf /etc/systemd/system/sweetie_bot_llm.service /etc/systemd/system/multi-user.target.wants/sweetie_bot_llm.service && \
    ln -sf /etc/systemd/system/sweetie_bot_classification.service /etc/systemd/system/multi-user.target.wants/sweetie_bot_classification.service && \
    ln -sf /etc/systemd/system/sweetie_bot_silero.service /etc/systemd/system/multi-user.target.wants/sweetie_bot_silero.service

# final cleanup
RUN find /opt -type d -name __pycache__ -prune -exec rm -rf {} + && \
    find /opt -type f \( -name '*.pyc' -o -name '*.pyo' -o -name '*.a' \) -delete && \
    rm -rf /root/.cache /tmp/* /var/tmp/*

EXPOSE 5000 5001 5002 5003 5004 5005 5006

CMD ["/usr/bin/systemctl"]
