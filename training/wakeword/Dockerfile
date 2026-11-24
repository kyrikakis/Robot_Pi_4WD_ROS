# Wake word training (openWakeWord) for Rainbow, run on the Mac in Docker (CPU).
# Not part of the robot image. See README.md.
FROM python:3.11.10-slim-bookworm

ARG OPENWAKEWORD_COMMIT=368c03716d1e92591906a84949bc477f3a834455
ARG PIPER_SAMPLE_GENERATOR_COMMIT=2971426a55072f7d22fec416ca7800df8bd23207
# Same onnxruntime as the robot image, so the evaluation scores match the robot's
ARG ONNXRUNTIME_VERSION=1.32.0

RUN apt-get update && apt-get install -y --no-install-recommends \
        git curl ffmpeg espeak-ng libsndfile1 && rm -rf /var/lib/apt/lists/*

# Every package pinned, dependencies included (--no-deps on the packages with loose
# or platform-specific requirements).
RUN pip install --no-cache-dir \
        torch==2.4.1 torchaudio==2.4.1 numpy==1.26.4 scipy==1.13.1 \
        onnxruntime==$ONNXRUNTIME_VERSION onnx==1.16.2 \
        soundfile==0.12.1 pyarrow==17.0.0 pyyaml==6.0.2 tqdm==4.66.5 requests==2.32.3 \
        scikit-learn==1.5.2 mutagen==1.47.0 torchinfo==1.8.0 torchmetrics==1.2.0 \
        audiomentations==0.33.0 julius==0.2.7 torch-pitch-shift==1.2.5 primePy==1.3 \
        pronouncing==0.2.0 cmudict==1.0.29 \
        hyperpyyaml==1.2.2 sentencepiece==0.2.0 joblib==1.4.2 huggingface_hub==0.25.2 \
        packaging==24.1 lightning-utilities==0.11.7 \
        matplotlib==3.9.2 contourpy==1.3.0 cycler==0.12.1 fonttools==4.54.1 kiwisolver==1.4.7 \
        pillow==10.4.0 pyparsing==3.1.4 && \
    pip install --no-cache-dir --no-deps \
        torch-audiomentations==0.11.1 speechbrain==0.5.16 acoustics==0.2.6 \
        piper-tts==1.3.0 tabulate==0.9.0 pandas==2.2.3 python-dateutil==2.9.0.post0 \
        pytz==2024.2 tzdata==2024.2 six==1.16.0 \
        pyroomacoustics==0.10.1 Cython==3.3.1

RUN git clone https://github.com/dscripka/openWakeWord /opt/openWakeWord && \
    git -C /opt/openWakeWord checkout $OPENWAKEWORD_COMMIT && \
    pip install --no-cache-dir --no-deps -e /opt/openWakeWord && \
    # train.py's flags default to the string "False", which is true: without this it
    # always tries the .tflite export (TensorFlow, not installed; we use the .onnx)
    sed -i 's/default="False"/default=False/' /opt/openWakeWord/openwakeword/train.py && \
    git clone https://github.com/rhasspy/piper-sample-generator /opt/piper-sample-generator && \
    git -C /opt/piper-sample-generator checkout $PIPER_SAMPLE_GENERATOR_COMMIT

# openWakeWord's feature models (the same files and checksums as the robot image)
RUN cd /opt/openWakeWord/openwakeword && mkdir -p resources/models && cd resources/models && \
    curl -fsSLO https://github.com/dscripka/openWakeWord/releases/download/v0.5.1/melspectrogram.onnx && \
    curl -fsSLO https://github.com/dscripka/openWakeWord/releases/download/v0.5.1/embedding_model.onnx && \
    curl -fsSLO https://github.com/dscripka/openWakeWord/releases/download/v0.5.1/hey_jarvis_v0.1.onnx && \
    echo "ba2b0e0f8b7b875369a2c89cb13360ff53bac436f2895cced9f479fa65eb176f  melspectrogram.onnx" > sums && \
    echo "70d164290c1d095d1d4ee149bc5e00543250a7316b59f31d056cff7bd3075c1f  embedding_model.onnx" >> sums && \
    echo "94a13cfe60075b132f6a472e7e462e8123ee70861bc3fb58434a73712ee0d2cb  hey_jarvis_v0.1.onnx" >> sums && \
    sha256sum -c sums && rm sums

ENV PYTHONUNBUFFERED=1
WORKDIR /work
