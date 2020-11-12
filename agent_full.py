#!/usr/bin/env python3
from aido_agents import FullAgent
from aido_schemas import protocol_agent_DB20_fullstate, wrap_direct


def main() -> None:
    node = FullAgent()
    protocol = protocol_agent_DB20_fullstate
    wrap_direct(node=node, protocol=protocol)


if __name__ == "__main__":
    main()
