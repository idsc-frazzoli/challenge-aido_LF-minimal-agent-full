ARG AIDO_REGISTRY
FROM ${AIDO_REGISTRY}/duckietown/aido-base-python3:daffy-aido4

COPY requirements.* ./
RUN pip install -r requirements.resolved
RUN pip list

COPY . .

RUN PYTHONPATH=. python -c "from agent_full import *"

ENTRYPOINT ["python3", "agent_full.py"]
