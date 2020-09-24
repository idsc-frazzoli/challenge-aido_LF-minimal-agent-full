ARG AIDO_REGISTRY
FROM ${AIDO_REGISTRY}/duckietown/aido-base-python3:daffy

ARG PIP_INDEX_URL
ENV PIP_INDEX_URL=${PIP_INDEX_URL}

COPY requirements.* ./
RUN pip install -U pip>=20.2
RUN pip install --use-feature=2020-resolver -r requirements.resolved
RUN pip list

COPY . .

RUN PYTHONPATH=. python -c "from agent_full import *"

ENTRYPOINT ["python3", "agent_full.py"]
