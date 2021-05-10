ARG AIDO_REGISTRY
FROM ${AIDO_REGISTRY}/duckietown/aido-base-python3:daffy-amd64

ARG PIP_INDEX_URL
ENV PIP_INDEX_URL=${PIP_INDEX_URL}

RUN pip3 install -U "pip>=20.2"
COPY requirements.* ./
RUN cat requirements.* > .requirements.txt
RUN pip3 install --use-feature=2020-resolver -r .requirements.txt
# fixme why this line?
RUN pip3 uninstall dataclasses -y

COPY . .

RUN python setup.py install

RUN PYTHONPATH=. python -c "from agent_full import *"

ENTRYPOINT ["python3", "agent_full.py"]
