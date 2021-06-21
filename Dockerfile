ARG AIDO_REGISTRY
FROM ${AIDO_REGISTRY}/duckietown/aido-base-python3:daffy-amd64

ARG PIP_INDEX_URL
ENV PIP_INDEX_URL=${PIP_INDEX_URL}

COPY requirements.* ./
RUN cat requirements.* > .requirements.txt
RUN python3 -m pip  install -r .requirements.txt

COPY . .

RUN python3 -m pip install .
# fixme why this line? - some packages install dataclasses (an old compat package)
RUN python3 -m pip  uninstall dataclasses -y

RUN node-launch --config node_launch.yaml --check

ENTRYPOINT ["node-launch", "--config", "node_launch.yaml"]
